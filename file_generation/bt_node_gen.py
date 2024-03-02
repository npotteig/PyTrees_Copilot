from typing import ClassVar, Tuple, Dict

from jinja2 import Environment

from base_gen import Generator
from header_extractor import ExternVarMap, FunctionMap

class BTNodeGenerator(Generator):
    _template_file: ClassVar[str] = "bt_node_template.j2"
    
    _msg_type_map: ClassVar[Dict[str, str]] = {
        "bool": "std_msgs.msg.Bool",
        "uint16_t": "std_msgs.msg.UInt16",
        "uint8_t": "std_msgs.msg.UInt8",
        "uint32_t": "std_msgs.msg.UInt32",
        "uint64_t": "std_msgs.msg.UInt64",
        "int8_t": "std_msgs.msg.Int8",
        "int16_t": "std_msgs.msg.Int16",
        "int32_t": "std_msgs.msg.Int32",
        "int64_t": "std_msgs.msg.Int64",
        "float": "std_msgs.msg.Float32",
        "double": "std_msgs.msg.Float64",
    }
    
    @classmethod
    def generate(cls, input_data: Tuple[ExternVarMap, FunctionMap], output_file: str, j2_env: Environment) -> None:
        bt_node_file = output_file
        template = j2_env.get_template(cls._template_file)
        
        extern_var_map, function_map = input_data
        
        input_dict = {
            "externs": [],
            "handlers": [],
        }
        
        for name, var_type in extern_var_map.items():
            input_dict["externs"].append((
                name,
                "copilot/" + name,
                cls._msg_type_map[var_type]
            ))
        
        for name, params in function_map.items():
            if len(params) == 1 and params[0][1] == "void":
                continue
            input_dict["handlers"].append((
                name,
                "copilot/" + name
            ))
        
        with open(bt_node_file, "w") as f:
            f.write(template.render(input_dict))