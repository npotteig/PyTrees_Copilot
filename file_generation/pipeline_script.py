import os
import argparse

from monitor_node_gen import MonitorNodeGenerator
from bt_node_gen import BTNodeGenerator
from jinja2 import Environment, FileSystemLoader

from header_extractor import HeaderExtractor

TEMPLATE_DIR = "./file_generation/templates"

if __name__ == "__main__":
    # Set up command line argument parsing
    parser = argparse.ArgumentParser(description="Generate ROS2 nodes from a Copilot Monitor")
    parser.add_argument("--header_file", help="Path to the file containing monitor.h header file")
    parser.add_argument("--cpp_dir", help="Path to the directory to save the monitor node")
    parser.add_argument("--py_dir", help="Path to the directory to save the bt node")
    
    
    args = parser.parse_args()
    
    extracted_data = HeaderExtractor.extract(args.header_file)
    env = Environment(loader=FileSystemLoader(TEMPLATE_DIR))
    MonitorNodeGenerator.generate(extracted_data, os.path.join(args.cpp_dir, 'monitor_node.cpp'), env)
    BTNodeGenerator.generate(extracted_data, os.path.join(args.py_dir, 'bt_node.py'), env)