#! /usr/bin/python
import os
import json
import jsonschema
from jsonschema import validate
from pathlib import Path

# Load schema
schema = json.loads(Path(os.path.dirname(os.path.abspath(__file__)) + "/parts_list_schema.json").read_text())

# Load parts list
parts = json.loads(Path(os.path.dirname(os.path.abspath(__file__)) + "/parts_list.json").read_text())

try:
    jsonschema.validate(instance=parts, schema=schema)
    print("✅ parts_list.json is valid")
except jsonschema.exceptions.ValidationError as e:
    print("❌ Validation error:", e.message)
    print("At path:", list(e.path))   # e.g. ['nut_size']