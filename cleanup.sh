#!/bin/bash
#
# cleanup.sh - Clean up temporary and compiled files in the Rincewind project
#
# This script removes temporary files, Python bytecode files, and other
# unnecessary files to reduce directory size and clean up the project.

echo "Starting Rincewind project cleanup..."

# Remove Python bytecode files
find . -name "*.pyc" -type f -delete
echo "✓ Removed Python bytecode files (*.pyc)"

# Remove temporary editor files
find . -name "*.swp" -type f -delete  # Vim swap files
find . -name "*~" -type f -delete     # Backup files
find . -name "*.bak" -type f -delete  # Backup files
echo "✓ Removed editor temporary files"

# Remove __pycache__ directories
find . -name "__pycache__" -type d -exec rm -rf {} +
if [ $? -ne 0 ]; then
    # If the above command fails, try this alternative
    find . -name "__pycache__" -type d -prune -exec rm -rf {} \;
fi
echo "✓ Removed __pycache__ directories"

# Remove .DS_Store files (common on macOS)
find . -name ".DS_Store" -type f -delete
echo "✓ Removed .DS_Store files"

# Remove Jupyter notebook checkpoints
find . -path "**/.ipynb_checkpoints" -type d -exec rm -rf {} +
if [ $? -ne 0 ]; then
    # If the above command fails, try this alternative
    find . -path "**/.ipynb_checkpoints" -type d -prune -exec rm -rf {} \;
fi
echo "✓ Removed Jupyter notebook checkpoints"

# Optional: Remove any log files in the project directory
find . -name "*.log" -type f -delete
echo "✓ Removed log files"

# Print summary of space saved (optional)
echo ""
echo "Cleanup complete! Directory is now lighter."

# Make the script executable
chmod +x "${0}"