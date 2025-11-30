#!/bin/bash
#
# Parse Package Information from opencog-debian update scripts
# This script extracts package metadata from the update-*.sh scripts
#
# Usage: parse-package-info.sh <component-name>
# Example: parse-package-info.sh cogutil
#

set -e

COMPONENT=$1

if [ -z "$COMPONENT" ]; then
    echo "Error: Component name required"
    echo "Usage: $0 <component-name>"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
UPDATE_SCRIPT="$REPO_ROOT/opencog-debian/$COMPONENT/update-$COMPONENT.sh"

if [ ! -f "$UPDATE_SCRIPT" ]; then
    echo "Error: Update script not found: $UPDATE_SCRIPT"
    exit 1
fi

# Extract information from the update script
REPO_NAME=$(grep '^REPO_NAME=' "$UPDATE_SCRIPT" | head -1 | cut -d'=' -f2)
DEB_NAME=$(grep '^DEB_NAME=' "$UPDATE_SCRIPT" | head -1 | cut -d'=' -f2)
VERSION=$(grep '^VERSION=' "$UPDATE_SCRIPT" | head -1 | cut -d'=' -f2)

# Output in a format that can be sourced by bash or used in GitHub Actions
echo "REPO_NAME=$REPO_NAME"
echo "DEB_NAME=$DEB_NAME"
echo "VERSION=$VERSION"
echo "DEBIAN_DIR=$REPO_ROOT/opencog-debian/$COMPONENT/debian"
echo "SOURCE_DIR=$REPO_ROOT/$COMPONENT"
