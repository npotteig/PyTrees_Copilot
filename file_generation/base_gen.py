from typing import Tuple
from abc import abstractmethod, ABC

from jinja2 import Environment
from header_extractor import ExternVarMap, FunctionMap

class Generator(ABC):
    
    
    @classmethod
    @abstractmethod
    def generate(cls, input_data: Tuple[ExternVarMap, FunctionMap], output_file: str, j2_env: Environment) -> None:
        ...