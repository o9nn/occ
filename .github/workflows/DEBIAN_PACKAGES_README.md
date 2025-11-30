# Debian Packages Workflow

## Overview

The `debian-packages.yml` workflow automates the build process for OpenCog Debian packages using `dpkg-buildpackage`. This workflow generates `.deb` packages for all major OpenCog components in the proper dependency order.

## Workflow Structure

The workflow builds packages in multiple stages, respecting the dependency hierarchy:

### Build Order

1. **cogutil** - Foundation utilities library
2. **atomspace** - Hypergraph knowledge representation (requires cogutil)
3. **unify** - Unification algorithms (requires cogutil, atomspace)
4. **ure** - Unified Rule Engine (requires cogutil, atomspace, unify)
5. **cogserver** - Network shell and server (requires cogutil, atomspace)
6. **attention** - ECAN attention allocation (requires cogutil, atomspace, cogserver)
7. **moses** - Evolutionary optimization (requires cogutil)
8. **asmoses** - MOSES with AtomSpace integration (requires moses, atomspace)
9. **miner** - Pattern mining (requires cogutil, atomspace, ure)
10. **pln** - Probabilistic Logic Networks (requires cogutil, atomspace, ure)

## Prerequisites Installed

Each build job installs the necessary prerequisites before building:

```bash
sudo apt-get install -y \
  debhelper \
  dpkg-dev \
  cmake \
  g++ \
  guile-3.0-dev \
  libboost-all-dev \
  binutils-dev \
  libiberty-dev
```

Additional component-specific dependencies are installed as needed.

## Build Process

For each component, the workflow:

1. **Checks out the repository** with all submodules
2. **Installs prerequisites** (system packages)
3. **Downloads previous packages** (artifacts from earlier stages)
4. **Installs dependencies** (OpenCog packages built in previous stages)
5. **Copies debian/ directory** from `opencog-debian/<component>/debian/`
6. **Builds the package** using `dpkg-buildpackage -us -uc -b`
7. **Uploads artifacts** for use in later stages

## Command Pattern

The core build command for each component follows this pattern:

```bash
cd <component>
dpkg-buildpackage -us -uc -b
```

Where:
- `-us`: Do not sign the source package
- `-uc`: Do not sign the .changes file
- `-b`: Build binary packages only

## Artifacts

### Individual Component Artifacts

Each component build stage uploads its generated packages:

- **cogutil-debian-packages**: cogutil library packages
- **atomspace-debian-packages**: atomspace library packages
- **unify-debian-packages**: unify library packages
- **ure-debian-packages**: URE packages
- **cogserver-debian-packages**: CogServer packages
- **attention-debian-packages**: Attention allocation packages
- **moses-debian-packages**: MOSES packages
- **asmoses-debian-packages**: ASMOSES packages
- **miner-debian-packages**: Pattern miner packages
- **pln-debian-packages**: PLN packages

Retention: 7 days

### Combined Artifact

The final summary job creates a combined artifact:

- **all-debian-packages**: All generated .deb files
- Retention: 30 days

## Installation

After downloading the artifacts from GitHub Actions:

```bash
# 1. Install cogutil (foundation)
sudo dpkg -i *cogutil*.deb

# 2. Install atomspace
sudo dpkg -i *atomspace*.deb

# 3. Install remaining packages
sudo dpkg -i *.deb

# 4. Fix any dependency issues
sudo apt-get install -f -y
```

## Triggering the Workflow

The workflow runs on:

- **Push** to `main` or `master` branches
- **Pull requests** to `main` or `master` branches
- **Manual dispatch** via GitHub Actions UI

### Manual Trigger

1. Go to Actions tab in GitHub
2. Select "Build Debian Packages" workflow
3. Click "Run workflow"
4. Select branch
5. Click "Run workflow" button

## Build Summary

After all builds complete, the workflow generates a summary showing:

- Total number of packages built
- List of all packages with sizes
- Installation instructions
- Component dependency information

This summary is available in the workflow run's summary page.

## Troubleshooting

### Build Failures

If a build fails:

1. Check the job logs for the failing component
2. Verify that `opencog-debian/<component>/debian/` directory exists
3. Ensure all dependencies are properly specified in `debian/control`
4. Check that previous stage packages were successfully built

### Missing debian/ Directory

If the workflow shows "⚠️ No debian/ directory found":

1. Ensure `opencog-debian/<component>/debian/` exists in the repository
2. Check that the directory contains required files:
   - `control`
   - `rules`
   - `changelog`
   - `compat`
   - `copyright`

### Dependency Issues

If package installation fails:

1. Check dependency order - earlier stages must complete successfully
2. Verify `Build-Depends` in `debian/control` files
3. Run `sudo apt-get install -f -y` to fix broken dependencies

## Component Details

### cogutil

**Dependencies**: None (foundation library)
**Packages**: `libcogutil`, `libcogutil-dev`
**Description**: Core utilities for OpenCog framework

### atomspace

**Dependencies**: cogutil
**Packages**: `libatomspace`, `libatomspace-dev`
**Description**: Hypergraph knowledge representation system

### unify

**Dependencies**: cogutil, atomspace
**Packages**: `opencog-unify`
**Description**: Unification algorithms for pattern matching

### ure

**Dependencies**: cogutil, atomspace, unify
**Packages**: `opencog-ure`
**Description**: Unified Rule Engine for forward/backward chaining

### cogserver

**Dependencies**: cogutil, atomspace
**Packages**: `opencog-cogserver`
**Description**: Network shell and server for AtomSpace

### attention

**Dependencies**: cogutil, atomspace, cogserver
**Packages**: `opencog-attention`
**Description**: ECAN attention allocation mechanism

### moses

**Dependencies**: cogutil
**Packages**: `opencog-moses`
**Description**: Machine learning and evolutionary optimization

### asmoses

**Dependencies**: moses, atomspace
**Packages**: `opencog-asmoses`
**Description**: MOSES with AtomSpace integration

### miner

**Dependencies**: cogutil, atomspace, ure
**Packages**: `opencog-miner`
**Description**: Pattern mining for knowledge discovery

### pln

**Dependencies**: cogutil, atomspace, ure
**Packages**: `opencog-pln`
**Description**: Probabilistic Logic Networks for uncertain reasoning

## Future Enhancements

Potential improvements to this workflow:

1. **Parallel Builds**: Build independent components in parallel
2. **Caching**: Cache apt packages and build artifacts
3. **Testing**: Add package installation and functionality tests
4. **Publishing**: Automatically publish to APT repository
5. **Matrix Strategy**: Build for multiple Ubuntu/Debian versions
6. **Signing**: Add GPG signing for packages
7. **Changelog**: Auto-generate changelog entries

## Related Documentation

- [OpenCog Debian README](../../opencog-debian/README.md)
- [Debian Packaging Summary](../../DEBIAN_PACKAGING_SUMMARY.md)
- [Build Instructions](../../BUILD_INSTRUCTIONS.md)

## License

This workflow and documentation are part of the OpenCog project and follow the same licensing terms (AGPL-3.0).
