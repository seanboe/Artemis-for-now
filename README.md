# NASA Artemis

For nasa 

DUE DATE: 1/15/22 Submit!!!


## Docs
for later

## other stuff

The Artemis core repo is a [monorepo](https://en.wikipedia.org/wiki/Monorepo)
managed by [Lerna](https://lerna.js.org/). Lerna is responsible for installing
the dependencies of the packages and tasks that belong in this repo. In general,
Cumulus's npm packages can be found in the [packages](./packages) directory, and
workflow tasks can be found in the [tasks](./tasks) directory.

To help cut down on the time and disk space required to install the dependencies
of the packages in this monorepo, all `devDependencies` are defined in the
top-level [package.json](./package.json). The
[Node module resolution algorithm](https://nodejs.org/api/modules.html#modules_loading_from_node_modules_folders)
allows all of the packages and tasks to find their dev dependencies in that
top-level `node_modules` directory.

