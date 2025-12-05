# Finetune gr00t to control a SO-100 robot arm

In this tutorial, we are going to train a Vision-Language Model (VLA) to control a SO-100 robot arm.

## Prerequisites

- You have recorded a dataset of the SO-100 robot arm in LeRobot dataset format (using phospho or LeRobot).
- You have a NVIDIA GPU.

If you don't have access to such a machine, you can rent a cloud instance with a GPU from any provider. We recommend getting an A-100 80GB.

## Installation and setup

First, if you haven't already, clone this repository:

```bash
git clone https://github.com/phospho-app/phosphobot.git
cd phosphobot
```

We are going to use the phospho fork of Isaac-GR00T repo that supports multiple SO-100 as embodiments.

Install the dependencies using conda:

```bash
git clone https://github.com/phospho-app/Isaac-GR00T.git
conda create -n gr00t python=3.10 -y
conda activate gr00t
pip install --upgrade setuptools
pip install -e Isaac-GR00T
pip install -e .
pip install --no-build-isolation flash-attn==2.7.1.post4
```

Now, you need to login to Hugging Face to be able to download the datasets.
Get an access token from [here](https://huggingface.co/settings/tokens) and set it as an environment variable:

```bash
export HF_TOKEN=<your_token>
```

Follow the instructions to login.

You should now have a working environment to finetune the model in `gr00t` conda environment.

## Finetune the model

In the phosphobot folder, still in the `gr00t` conda environment, run:

```bash
python scripts/gr00t/train.py --dataset-id YOUR-DATASET-ID
```

Or simply in pure python:

```python
from pathlib import Path
from phosphobot.am import Gr00tTrainerConfig, Gr00tTrainer

config = Gr00tTrainerConfig(
    # Path to the Isaac-GR00T repo
    path_to_gr00t_repo="Isaac-GR00T/",
    dataset_id="YOUR_USERNAME/YOUR_DATASET_ID",
)

trainer = Gr00tTrainer(config)
trainer.train()
```

Adjust the batch size to use as much as possible of the GPU memory.
If you encounter an Out of Memory error, try to reduce the batch size.

Use tools like `nvidia-smi` or `nvitop` to monitor the GPU memory usage.
Use `wandb` to track the training (just login with `wandb login`).

For more advanced usage, see all the options available with:

```bash
python scripts/gr00t/train.py --help
```

## Evaluate the finetuned model

TODO

## Test the finetuned model on your robot

### Start the inference server

Make sure the setup of your robot matches the dataset setup (number of cameras, order of cameras, etc...).

Run the inference server on the machine where the model is (the one with the GPU):

```bash
python scripts/gr00t/serve.py --model-path /outputs
```

### Make your local robot move

If you are using a remote machine, make sure the port (5555 by default) is open.

To control your local robot, we will use `phosphobot`.
If you haven't already, install it with:

```bash
curl -fsSL https://raw.githubusercontent.com/phospho-app/phosphobot/main/install.sh | sudo bash
```

Then, on your local machine, run:

```bash
phosphobot run
```

More information on how to use phosphobot [here](https://docs.phospho.ai/).

Now, in another terminal, you need to install the `torch` and `zmq` libraries on your local machine:

```bash
pip install torch zmq
```

Adapt the script `scripts/quickstart_ai_gr00t.py` to your specific setup (robot, cameras, phosphobot server and inference server).

Then run it:

```bash
python scripts/quickstart_ai_gr00t.py
```

Your robot is now controled by the finetuned model!

## Troubleshooting common issues

### Git LFS not installed

On your linux machine, run:

```bash
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
git lfs install
```

### Failed to initialize NVML

After running the installation commands, try restarting your machine:

```bash
sudo reboot
```

## Acknowledgements

- the NVIDIA team for providing the GR00T model and most of the code for the finetuning. Link to the original repo [here](https://github.com/NVIDIA/Isaac-GR00T).
- the LeRobot team for the dataset format. Link to the original repo [here](https://github.com/huggingface/lerobot).
