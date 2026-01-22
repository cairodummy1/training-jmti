#!/bin/bash
set -e

echo "=== Step 1: Uninstall old torch / torchvision ==="
pip uninstall -y torch torchvision || true
pip3 uninstall -y torch torchvision || true

echo "=== Step 2: Remove leftover torch files (user site) ==="
rm -rf ~/.local/lib/python3.10/site-packages/torch*
rm -rf ~/.local/lib/python3.10/site-packages/torchvision*

echo "=== Step 3: Remove PyTorch cache ==="
rm -rf ~/.cache/torch

echo "=== Step 4: Download NVIDIA wheels ==="
mkdir -p Downloads/jetson_torch_wheels
cd Downloads/jetson_torch_wheels

wget -O torch-2.3.0-cp310-cp310-linux_aarch64.whl \
  https://nvidia.box.com/shared/static/mp164asf3sceb570wvjsrezk1p4ftj8t.whl

wget -O torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl \
  https://nvidia.box.com/shared/static/xpr06qe6ql3l6rj22cu3c45tz1wzi36p.whl

echo "=== Step 5: Install NVIDIA torch ==="
pip install --no-cache ./torch-2.3.0-cp310-cp310-linux_aarch64.whl

echo "=== Step 6: Install NVIDIA torchvision ==="
pip install --no-cache ./torchvision-0.18.0a0+6043bc2-cp310-cp310-linux_aarch64.whl

echo "=== Step 7: Verify installation ==="
python3 - <<EOF
import torch, torchvision
print("torch:", torch.__version__)
print("torchvision:", torchvision.__version__)
print("cuda available:", torch.cuda.is_available())
EOF

echo "=== DONE ==="
