
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>

#include <soc/tegra/ahb.h>

#define DRV_NAME "tegra-ahb"

#define AHB_ARBITRATION_DISABLE		0x04
#define AHB_ARBITRATION_PRIORITY_CTRL	0x08
#define   AHB_PRIORITY_WEIGHT(x)	(((x) & 0x7) << 29)
#define   PRIORITY_SELECT_USB BIT(6)
#define   PRIORITY_SELECT_USB2 BIT(18)
#define   PRIORITY_SELECT_USB3 BIT(17)

#define AHB_GIZMO_AHB_MEM		0x10
#define   ENB_FAST_REARBITRATE BIT(2)
#define   DONT_SPLIT_AHB_WR     BIT(7)

#define AHB_GIZMO_APB_DMA		0x14
#define AHB_GIZMO_IDE			0x1c
#define AHB_GIZMO_USB			0x20
#define AHB_GIZMO_AHB_XBAR_BRIDGE	0x24
#define AHB_GIZMO_CPU_AHB_BRIDGE	0x28
#define AHB_GIZMO_COP_AHB_BRIDGE	0x2c
#define AHB_GIZMO_XBAR_APB_CTLR		0x30
#define AHB_GIZMO_VCP_AHB_BRIDGE	0x34
#define AHB_GIZMO_NAND			0x40
#define AHB_GIZMO_SDMMC4		0x48
#define AHB_GIZMO_XIO			0x4c
#define AHB_GIZMO_BSEV			0x64
#define AHB_GIZMO_BSEA			0x74
#define AHB_GIZMO_NOR			0x78
#define AHB_GIZMO_USB2			0x7c
#define AHB_GIZMO_USB3			0x80
#define   IMMEDIATE	BIT(18)

#define AHB_GIZMO_SDMMC1		0x84
#define AHB_GIZMO_SDMMC2		0x88
#define AHB_GIZMO_SDMMC3		0x8c
#define AHB_MEM_PREFETCH_CFG_X		0xdc
#define AHB_ARBITRATION_XBAR_CTRL	0xe0
#define AHB_MEM_PREFETCH_CFG3		0xe4
#define AHB_MEM_PREFETCH_CFG4		0xe8
#define AHB_MEM_PREFETCH_CFG1		0xf0
#define AHB_MEM_PREFETCH_CFG2		0xf4
#define   PREFETCH_ENB	BIT(31)
#define   MST_ID(x)	(((x) & 0x1f) << 26)
#define   AHBDMA_MST_ID	MST_ID(5)
#define   USB_MST_ID	MST_ID(6)
#define   USB2_MST_ID	MST_ID(18)
#define   USB3_MST_ID	MST_ID(17)
#define   ADDR_BNDRY(x)	(((x) & 0xf) << 21)
#define   INACTIVITY_TIMEOUT(x)	(((x) & 0xffff) << 0)

#define AHB_ARBITRATION_AHB_MEM_WRQUE_MST_ID	0xfc

#define AHB_ARBITRATION_XBAR_CTRL_SMMU_INIT_DONE BIT(17)

/*
 * INCORRECT_BASE_ADDR_LOW_BYTE: Legacy kernel DT files for Tegra SoCs
 * prior to Tegra124 generally use a physical base address ending in
 * 0x4 for the AHB IP block.  According to the TRM, the low byte
 * should be 0x0.  During device probing, this macro is used to detect
 * whether the passed-in physical address is incorrect, and if so, to
 * correct it.
 */
#define INCORRECT_BASE_ADDR_LOW_BYTE		0x4

static struct platform_driver tegra_ahb_driver;

static const u32 tegra_ahb_gizmo[] = {
	AHB_ARBITRATION_DISABLE,
	AHB_ARBITRATION_PRIORITY_CTRL,
	AHB_GIZMO_AHB_MEM,
	AHB_GIZMO_APB_DMA,
	AHB_GIZMO_IDE,
	AHB_GIZMO_USB,
	AHB_GIZMO_AHB_XBAR_BRIDGE,
	AHB_GIZMO_CPU_AHB_BRIDGE,
	AHB_GIZMO_COP_AHB_BRIDGE,
	AHB_GIZMO_XBAR_APB_CTLR,
	AHB_GIZMO_VCP_AHB_BRIDGE,
	AHB_GIZMO_NAND,
	AHB_GIZMO_SDMMC4,
	AHB_GIZMO_XIO,
	AHB_GIZMO_BSEV,
	AHB_GIZMO_BSEA,
	AHB_GIZMO_NOR,
	AHB_GIZMO_USB2,
	AHB_GIZMO_USB3,
	AHB_GIZMO_SDMMC1,
	AHB_GIZMO_SDMMC2,
	AHB_GIZMO_SDMMC3,
	AHB_MEM_PREFETCH_CFG_X,
	AHB_ARBITRATION_XBAR_CTRL,
	AHB_MEM_PREFETCH_CFG3,
	AHB_MEM_PREFETCH_CFG4,
	AHB_MEM_PREFETCH_CFG1,
	AHB_MEM_PREFETCH_CFG2,
	AHB_ARBITRATION_AHB_MEM_WRQUE_MST_ID,
};

struct tegra_ahb {
	void __iomem	*regs;
	struct device	*dev;
	u32		ctx[];
};

static inline u32 gizmo_readl(struct tegra_ahb *ahb, u32 offset)
{
	return readl(ahb->regs + offset);
}

static inline void gizmo_writel(struct tegra_ahb *ahb, u32 value, u32 offset)
{
	writel(value, ahb->regs + offset);
}

#ifdef CONFIG_TEGRA_IOMMU_SMMU
int tegra_ahb_enable_smmu(struct device_node *dn)
{
	struct device *dev;
	u32 val;
	struct tegra_ahb *ahb;

	dev = driver_find_device_by_of_node(&tegra_ahb_driver.driver, dn);
	if (!dev)
		return -EPROBE_DEFER;
	ahb = dev_get_drvdata(dev);
	val = gizmo_readl(ahb, AHB_ARBITRATION_XBAR_CTRL);
	val |= AHB_ARBITRATION_XBAR_CTRL_SMMU_INIT_DONE;
	gizmo_writel(ahb, val, AHB_ARBITRATION_XBAR_CTRL);
	return 0;
}
EXPORT_SYMBOL(tegra_ahb_enable_smmu);
#endif

static int __maybe_unused tegra_ahb_suspend(struct device *dev)
{
	int i;
	struct tegra_ahb *ahb = dev_get_drvdata(dev);

	for (i = 0; i < ARRAY_SIZE(tegra_ahb_gizmo); i++)
		ahb->ctx[i] = gizmo_readl(ahb, tegra_ahb_gizmo[i]);
	return 0;
}

static int __maybe_unused tegra_ahb_resume(struct device *dev)
{
	int i;
	struct tegra_ahb *ahb = dev_get_drvdata(dev);

	for (i = 0; i < ARRAY_SIZE(tegra_ahb_gizmo); i++)
		gizmo_writel(ahb, ahb->ctx[i], tegra_ahb_gizmo[i]);
	return 0;
}

static UNIVERSAL_DEV_PM_OPS(tegra_ahb_pm,
			    tegra_ahb_suspend,
			    tegra_ahb_resume, NULL);

static void tegra_ahb_gizmo_init(struct tegra_ahb *ahb)
{
	u32 val;

	val = gizmo_readl(ahb, AHB_GIZMO_AHB_MEM);
	val |= ENB_FAST_REARBITRATE | IMMEDIATE | DONT_SPLIT_AHB_WR;
	gizmo_writel(ahb, val, AHB_GIZMO_AHB_MEM);

	val = gizmo_readl(ahb, AHB_GIZMO_USB);
	val |= IMMEDIATE;
	gizmo_writel(ahb, val, AHB_GIZMO_USB);

	val = gizmo_readl(ahb, AHB_GIZMO_USB2);
	val |= IMMEDIATE;
	gizmo_writel(ahb, val, AHB_GIZMO_USB2);

	val = gizmo_readl(ahb, AHB_GIZMO_USB3);
	val |= IMMEDIATE;
	gizmo_writel(ahb, val, AHB_GIZMO_USB3);

	val = gizmo_readl(ahb, AHB_ARBITRATION_PRIORITY_CTRL);
	val |= PRIORITY_SELECT_USB |
		PRIORITY_SELECT_USB2 |
		PRIORITY_SELECT_USB3 |
		AHB_PRIORITY_WEIGHT(7);
	gizmo_writel(ahb, val, AHB_ARBITRATION_PRIORITY_CTRL);

	val = gizmo_readl(ahb, AHB_MEM_PREFETCH_CFG1);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB |
		AHBDMA_MST_ID |
		ADDR_BNDRY(0xc) |
		INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(ahb, val, AHB_MEM_PREFETCH_CFG1);

	val = gizmo_readl(ahb, AHB_MEM_PREFETCH_CFG2);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB |
		USB_MST_ID |
		ADDR_BNDRY(0xc) |
		INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(ahb, val, AHB_MEM_PREFETCH_CFG2);

	val = gizmo_readl(ahb, AHB_MEM_PREFETCH_CFG3);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB |
		USB3_MST_ID |
		ADDR_BNDRY(0xc) |
		INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(ahb, val, AHB_MEM_PREFETCH_CFG3);

	val = gizmo_readl(ahb, AHB_MEM_PREFETCH_CFG4);
	val &= ~MST_ID(~0);
	val |= PREFETCH_ENB |
		USB2_MST_ID |
		ADDR_BNDRY(0xc) |
		INACTIVITY_TIMEOUT(0x1000);
	gizmo_writel(ahb, val, AHB_MEM_PREFETCH_CFG4);
}

static int tegra_ahb_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct tegra_ahb *ahb;
	size_t bytes;

	bytes = sizeof(*ahb) + sizeof(u32) * ARRAY_SIZE(tegra_ahb_gizmo);
	ahb = devm_kzalloc(&pdev->dev, bytes, GFP_KERNEL);
	if (!ahb)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	/* Correct the IP block base address if necessary */
	if (res &&
	    (res->start & INCORRECT_BASE_ADDR_LOW_BYTE) ==
	    INCORRECT_BASE_ADDR_LOW_BYTE) {
		dev_warn(&pdev->dev, "incorrect AHB base address in DT data - enabling workaround\n");
		res->start -= INCORRECT_BASE_ADDR_LOW_BYTE;
	}

	ahb->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ahb->regs))
		return PTR_ERR(ahb->regs);

	ahb->dev = &pdev->dev;
	platform_set_drvdata(pdev, ahb);
	tegra_ahb_gizmo_init(ahb);
	return 0;
}

static const struct of_device_id tegra_ahb_of_match[] = {
	{ .compatible = "nvidia,tegra30-ahb", },
	{ .compatible = "nvidia,tegra20-ahb", },
	{},
};

static struct platform_driver tegra_ahb_driver = {
	.probe = tegra_ahb_probe,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = tegra_ahb_of_match,
		.pm = &tegra_ahb_pm,
	},
};
module_platform_driver(tegra_ahb_driver);

static int rk_crypto_enable_clk(struct rk_crypto_info *dev)
{
	int err;

	err = clk_prepare_enable(dev->sclk);
	if (err) {
		dev_err(dev->dev, "[%s:%d], Couldn't enable clock sclk\n",
			__func__, __LINE__);
		goto err_return;
	}
	err = clk_prepare_enable(dev->aclk);
	if (err) {
		dev_err(dev->dev, "[%s:%d], Couldn't enable clock aclk\n",
			__func__, __LINE__);
		goto err_aclk;
	}
	err = clk_prepare_enable(dev->hclk);
	if (err) {
		dev_err(dev->dev, "[%s:%d], Couldn't enable clock hclk\n",
			__func__, __LINE__);
		goto err_hclk;
	}
	err = clk_prepare_enable(dev->dmaclk);
	if (err) {
		dev_err(dev->dev, "[%s:%d], Couldn't enable clock dmaclk\n",
			__func__, __LINE__);
		goto err_dmaclk;
	}
	return err;
err_dmaclk:
	clk_disable_unprepare(dev->hclk);
err_hclk:
	clk_disable_unprepare(dev->aclk);
err_aclk:
	clk_disable_unprepare(dev->sclk);
err_return:
	return err;
}

static void rk_crypto_disable_clk(struct rk_crypto_info *dev)
{
	clk_disable_unprepare(dev->dmaclk);
	clk_disable_unprepare(dev->hclk);
	clk_disable_unprepare(dev->aclk);
	clk_disable_unprepare(dev->sclk);
}

static int check_alignment(struct scatterlist *sg_src,
			   struct scatterlist *sg_dst,
			   int align_mask)
{
	int in, out, align;

	in = IS_ALIGNED((uint32_t)sg_src->offset, 4) &&
	     IS_ALIGNED((uint32_t)sg_src->length, align_mask);
	if (!sg_dst)
		return in;
	out = IS_ALIGNED((uint32_t)sg_dst->offset, 4) &&
	      IS_ALIGNED((uint32_t)sg_dst->length, align_mask);
	align = in && out;

	return (align && (sg_src->length == sg_dst->length));
}

static int rk_load_data(struct rk_crypto_info *dev,
			struct scatterlist *sg_src,
			struct scatterlist *sg_dst)
{
	unsigned int count;

	dev->aligned = dev->aligned ?
		check_alignment(sg_src, sg_dst, dev->align_size) :
		dev->aligned;
	if (dev->aligned) {
		count = min(dev->left_bytes, sg_src->length);
		dev->left_bytes -= count;

		if (!dma_map_sg(dev->dev, sg_src, 1, DMA_TO_DEVICE)) {
			dev_err(dev->dev, "[%s:%d] dma_map_sg(src)  error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		dev->addr_in = sg_dma_address(sg_src);

		if (sg_dst) {
			if (!dma_map_sg(dev->dev, sg_dst, 1, DMA_FROM_DEVICE)) {
				dev_err(dev->dev,
					"[%s:%d] dma_map_sg(dst)  error\n",
					__func__, __LINE__);
				dma_unmap_sg(dev->dev, sg_src, 1,
					     DMA_TO_DEVICE);
				return -EINVAL;
			}
			dev->addr_out = sg_dma_address(sg_dst);
		}
	} else {
		count = (dev->left_bytes > PAGE_SIZE) ?
			PAGE_SIZE : dev->left_bytes;

		if (!sg_pcopy_to_buffer(dev->first, dev->src_nents,
					dev->addr_vir, count,
					dev->total - dev->left_bytes)) {
			dev_err(dev->dev, "[%s:%d] pcopy err\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		dev->left_bytes -= count;
		sg_init_one(&dev->sg_tmp, dev->addr_vir, count);
		if (!dma_map_sg(dev->dev, &dev->sg_tmp, 1, DMA_TO_DEVICE)) {
			dev_err(dev->dev, "[%s:%d] dma_map_sg(sg_tmp)  error\n",
				__func__, __LINE__);
			return -ENOMEM;
		}
		dev->addr_in = sg_dma_address(&dev->sg_tmp);

		if (sg_dst) {
			if (!dma_map_sg(dev->dev, &dev->sg_tmp, 1,
					DMA_FROM_DEVICE)) {
				dev_err(dev->dev,
					"[%s:%d] dma_map_sg(sg_tmp)  error\n",
					__func__, __LINE__);
				dma_unmap_sg(dev->dev, &dev->sg_tmp, 1,
					     DMA_TO_DEVICE);
				return -ENOMEM;
			}
			dev->addr_out = sg_dma_address(&dev->sg_tmp);
		}
	}
	dev->count = count;
	return 0;
}

static void rk_unload_data(struct rk_crypto_info *dev)
{
	struct scatterlist *sg_in, *sg_out;

	sg_in = dev->aligned ? dev->sg_src : &dev->sg_tmp;
	dma_unmap_sg(dev->dev, sg_in, 1, DMA_TO_DEVICE);

	if (dev->sg_dst) {
		sg_out = dev->aligned ? dev->sg_dst : &dev->sg_tmp;
		dma_unmap_sg(dev->dev, sg_out, 1, DMA_FROM_DEVICE);
	}
}

static irqreturn_t rk_crypto_irq_handle(int irq, void *dev_id)
{
	struct rk_crypto_info *dev  = platform_get_drvdata(dev_id);
	u32 interrupt_status;

	spin_lock(&dev->lock);
	interrupt_status = CRYPTO_READ(dev, RK_CRYPTO_INTSTS);
	CRYPTO_WRITE(dev, RK_CRYPTO_INTSTS, interrupt_status);

	if (interrupt_status & 0x0a) {
		dev_warn(dev->dev, "DMA Error\n");
		dev->err = -EFAULT;
	}
	tasklet_schedule(&dev->done_task);

	spin_unlock(&dev->lock);
	return IRQ_HANDLED;
}

static int rk_crypto_enqueue(struct rk_crypto_info *dev,
			      struct crypto_async_request *async_req)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&dev->lock, flags);
	ret = crypto_enqueue_request(&dev->queue, async_req);
	if (dev->busy) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return ret;
	}
	dev->busy = true;
	spin_unlock_irqrestore(&dev->lock, flags);
	tasklet_schedule(&dev->queue_task);

	return ret;
}

static void rk_crypto_queue_task_cb(unsigned long data)
{
	struct rk_crypto_info *dev = (struct rk_crypto_info *)data;
	struct crypto_async_request *async_req, *backlog;
	unsigned long flags;
	int err = 0;

	dev->err = 0;
	spin_lock_irqsave(&dev->lock, flags);
	backlog   = crypto_get_backlog(&dev->queue);
	async_req = crypto_dequeue_request(&dev->queue);

	if (!async_req) {
		dev->busy = false;
		spin_unlock_irqrestore(&dev->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&dev->lock, flags);

	if (backlog) {
		backlog->complete(backlog, -EINPROGRESS);
		backlog = NULL;
	}

	dev->async_req = async_req;
	err = dev->start(dev);
	if (err)
		dev->complete(dev->async_req, err);
}

static void rk_crypto_done_task_cb(unsigned long data)
{
	struct rk_crypto_info *dev = (struct rk_crypto_info *)data;

	if (dev->err) {
		dev->complete(dev->async_req, dev->err);
		return;
	}

	dev->err = dev->update(dev);
	if (dev->err)
		dev->complete(dev->async_req, dev->err);
}

static struct rk_crypto_tmp *rk_cipher_algs[] = {
	&rk_ecb_aes_alg,
	&rk_cbc_aes_alg,
	&rk_ecb_des_alg,
	&rk_cbc_des_alg,
	&rk_ecb_des3_ede_alg,
	&rk_cbc_des3_ede_alg,
	&rk_ahash_sha1,
	&rk_ahash_sha256,
	&rk_ahash_md5,
};

static int rk_crypto_register(struct rk_crypto_info *crypto_info)
{
	unsigned int i, k;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(rk_cipher_algs); i++) {
		rk_cipher_algs[i]->dev = crypto_info;
		if (rk_cipher_algs[i]->type == ALG_TYPE_CIPHER)
			err = crypto_register_skcipher(
					&rk_cipher_algs[i]->alg.skcipher);
		else
			err = crypto_register_ahash(
					&rk_cipher_algs[i]->alg.hash);
		if (err)
			goto err_cipher_algs;
	}
	return 0;

err_cipher_algs:
	for (k = 0; k < i; k++) {
		if (rk_cipher_algs[i]->type == ALG_TYPE_CIPHER)
			crypto_unregister_skcipher(&rk_cipher_algs[k]->alg.skcipher);
		else
			crypto_unregister_ahash(&rk_cipher_algs[i]->alg.hash);
	}
	return err;
}

static void rk_crypto_unregister(void)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(rk_cipher_algs); i++) {
		if (rk_cipher_algs[i]->type == ALG_TYPE_CIPHER)
			crypto_unregister_skcipher(&rk_cipher_algs[i]->alg.skcipher);
		else
			crypto_unregister_ahash(&rk_cipher_algs[i]->alg.hash);
	}
}
