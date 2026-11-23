"""Create an input-dependent ONNX test fixture, NOT a trained scene model."""
from pathlib import Path

def varint(value):
    result = bytearray()
    while value > 127:
        result.append((value & 127) | 128)
        value >>= 7
    result.append(value)
    return bytes(result)

def integer(field, value):
    return varint(field << 3) + varint(value)

def message(field, value):
    if isinstance(value, str):
        value = value.encode("utf-8")
    return varint((field << 3) | 2) + varint(len(value)) + value

def tensor_info(name, dimensions):
    shape = b"".join(message(1, integer(1, d)) for d in dimensions)
    tensor_type = integer(1, 1) + message(2, shape)
    return message(1, name) + message(2, message(1, tensor_type))

def node(kind, source, target):
    return message(1, source) + message(2, target) + message(4, kind)

graph = (
    message(1, node("GlobalAveragePool", "input", "pool"))
    + message(1, node("Flatten", "pool", "logits"))
    + message(2, "test_only_rgb_channel_mean")
    + message(11, tensor_info("input", [1, 3, 4, 4]))
    + message(12, tensor_info("logits", [1, 3]))
)
model = integer(1, 8) + message(2, "perception-test-fixture") + message(7, graph) + message(8, integer(2, 13))
destination = Path(__file__).parent / "fixtures" / "scene_cls_rgb_mean.onnx"
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_bytes(model)
print(destination)
