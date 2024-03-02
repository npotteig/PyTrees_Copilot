from typing import TypeAlias, Dict, Tuple, List, Optional
from pycparser import parse_file, c_ast

ExternVarMap: TypeAlias = Dict[str, str]
FunctionMap: TypeAlias = Dict[str, List[Tuple[Optional[str], str]]]

class HeaderExtractor:
    
    @classmethod
    def extract(cls, header_file: str) -> Tuple[ExternVarMap, FunctionMap]:
        ast = parse_file(header_file)
        
        ExternVarMap = {}
        FunctionMap = {}
        
        for ext in ast.ext:
            if isinstance(ext.type, c_ast.TypeDecl):
                ExternVarMap[ext.name] = ext.type.type.names[0]
            elif isinstance(ext.type, c_ast.FuncDecl):
                params_list = []
                for param in ext.type.args.params:
                    params_list.append((param.name, param.type.type.names[0]))
                FunctionMap[ext.name] = params_list
            else:
                raise TypeError(f"Unsupported type {type(ext.type)}")
        
        return ExternVarMap, FunctionMap