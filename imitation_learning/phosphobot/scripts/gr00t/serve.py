import tyro
from argparse import Namespace
from dataclasses import dataclass

from gr00t.experiment.data_config import (
    ConfigGeneratorFromNames,  # type: ignore
)
from gr00t.model.policy import Gr00tPolicy  # type: ignore
from phosphobot.am.gr00t import RobotInferenceServer
from phosphobot.am.gr00t import Gr00tN1, Gr00tSpawnConfig
from loguru import logger


@dataclass
class Config:
    output_dir: str = "outputs/"
    """the directory to load the model from (or save the model to if downloading)"""

    server_port: int = 5555
    """the port to serve the model on"""

    hf_model_id: str | None = None
    """the Hugging Face id of the model (if None, will load the model from the output directory)"""


def main(config: Config):
    if config.hf_model_id is not None:
        # Download the model from Hugging Face
        raise NotImplementedError(
            "Downloading from Hugging Face is not implemented yet"
        )
    else:
        # Load the model from the output directory
        logger.info(f"Loading model from {config.output_dir}")

    # Get the model specifics
    model_specifics: Gr00tSpawnConfig = Gr00tN1.fetch_spawn_config(
        model_id=config.output_dir,
    )

    args = Namespace(
        model_path=config.output_dir,
        embodiment_tag=model_specifics.embodiment_tag,
        server=True,
        client=False,
        host="0.0.0.0",
        port=config.server_port,
        denoising_steps=4,
    )

    data_config = ConfigGeneratorFromNames(
        video_keys=model_specifics.video_keys,
        state_keys=model_specifics.state_keys,
        action_keys=model_specifics.action_keys,
    )
    modality_config = data_config.modality_config()  # type: ignore
    modality_transform = data_config.transform()  # type: ignore

    policy = Gr00tPolicy(
        model_path=args.model_path,
        modality_config=modality_config,
        modality_transform=modality_transform,
        embodiment_tag=args.embodiment_tag,
        denoising_steps=args.denoising_steps,
    )

    # Start the server
    server = RobotInferenceServer(model=policy, port=args.port)

    logger.info(
        f"Server starting on port {args.port}... (make sure the port is open and not already in use)"
    )

    server.run()


if __name__ == "__main__":
    config = tyro.cli(Config)

    main(config)
