import importlib.metadata

try:
    __version__ = importlib.metadata.version("phosphobot")
except importlib.metadata.PackageNotFoundError:
    print("PackageNotFoundError: No package metadata was found for 'phosphobot'.")
    __version__ = "unknown"
