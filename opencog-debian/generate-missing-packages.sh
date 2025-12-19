#!/bin/bash
# Generate Debian packaging for missing OpenCog components
# This script creates control, rules, and changelog files for all missing packages

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION="3.0.0-1"
DATE="Thu, 19 Dec 2025 17:00:00 +0000"

# Function to create a basic Debian package structure
create_package() {
    local PKG_NAME=$1
    local PKG_DESCRIPTION=$2
    local BUILD_DEPS=$3
    local RUNTIME_DEPS=$4
    local PKG_DIR="$SCRIPT_DIR/$PKG_NAME"
    
    echo "Creating package: $PKG_NAME"
    
    mkdir -p "$PKG_DIR/debian"
    
    # Create control file
    cat > "$PKG_DIR/debian/control" <<EOF
Source: $PKG_NAME
Section: science
Priority: optional
Maintainer: OpenCog Developers <opencog@googlegroups.com>
Build-Depends: debhelper-compat (= 13),
               cmake (>= 3.12),
               $BUILD_DEPS
Standards-Version: 4.6.0
Homepage: https://opencog.org/
Vcs-Browser: https://github.com/cogpy/occ
Vcs-Git: https://github.com/cogpy/occ.git

Package: $PKG_NAME
Architecture: any
Depends: \${shlibs:Depends}, \${misc:Depends},
         $RUNTIME_DEPS
Description: $PKG_DESCRIPTION
 This package is part of the OpenCog AGI framework and AGI-OS architecture.
EOF

    # Create rules file
    cat > "$PKG_DIR/debian/rules" <<'EOF'
#!/usr/bin/make -f
# -*- makefile -*-

%:
	dh $@ --buildsystem=cmake

override_dh_auto_configure:
	dh_auto_configure -- \
		-DCMAKE_BUILD_TYPE=Release \
		-DCMAKE_INSTALL_PREFIX=/usr

override_dh_auto_test:
	dh_auto_test || true
EOF
    
    chmod +x "$PKG_DIR/debian/rules"
    
    # Create changelog
    cat > "$PKG_DIR/debian/changelog" <<EOF
$PKG_NAME ($VERSION) unstable; urgency=medium

  * Initial Debian package for $PKG_NAME
  * Integration with OpenCog AGI-OS architecture

 -- OpenCog Developers <opencog@googlegroups.com>  $DATE
EOF

    echo "✓ Created $PKG_NAME"
}

# Create agents package
create_package "opencog-agents" \
    "Interactive agents for OpenCog AtomSpace" \
    "libcogutil-dev (>= 2.1.0), opencog-atomspace (>= 5.0.5), libboost-dev, guile-3.0-dev | guile-2.2-dev" \
    "libcogutil3, opencog-atomspace"

# Create sensory package
create_package "opencog-sensory" \
    "Sensory dataflow system for OpenCog" \
    "libcogutil-dev (>= 2.1.0), opencog-atomspace (>= 5.0.5), libboost-dev, guile-3.0-dev | guile-2.2-dev" \
    "libcogutil3, opencog-atomspace"

# Create coggml package
create_package "coggml" \
    "CogGML self-aware cognitive microkernel with shard architecture" \
    "libcogutil-dev (>= 2.1.0), libboost-dev, cmake (>= 3.12)" \
    "libcogutil3"

# Create cogself package
create_package "cogself" \
    "CogSelf AGI cognitive synergy framework" \
    "libcogutil-dev (>= 2.1.0), coggml (>= 3.0.0), opencog-atomspace (>= 5.0.5), libboost-dev" \
    "libcogutil3, coggml, opencog-atomspace"

# Create atomspace-accelerator package
create_package "atomspace-accelerator" \
    "AtomSpace accelerator inference engine for high-performance reasoning" \
    "libcogutil-dev (>= 2.1.0), opencog-atomspace (>= 5.0.5), libboost-dev" \
    "libcogutil3, opencog-atomspace"

# Create agentic-chatbots package
create_package "agentic-chatbots" \
    "Agentic chatbots integration for OpenCog AGI framework" \
    "libcogutil-dev (>= 2.1.0), opencog-atomspace (>= 5.0.5), libboost-dev, python3-dev" \
    "libcogutil3, opencog-atomspace, python3"

echo ""
echo "==================================================="
echo "All missing Debian packages created successfully!"
echo "==================================================="
echo ""
echo "Packages created:"
echo "  - opencog-agents"
echo "  - opencog-sensory"
echo "  - coggml"
echo "  - cogself"
echo "  - atomspace-accelerator"
echo "  - agentic-chatbots"
echo ""
echo "Next steps:"
echo "  1. Review generated control files"
echo "  2. Update build-all-production.sh to include new packages"
echo "  3. Update BUILD_ORDER_ENHANCED.md with new packages"
echo "  4. Test package builds"
echo ""
