import onnxruntime as ort

# Load the ONNX model
sess = ort.InferenceSession("../models/assembly_system_detector.onnx", providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])

# Get the name of the first input of the model
input_name = sess.get_inputs()[0].name
print(f"Input Name: {input_name}")

# Print the input shape
input_shape = sess.get_inputs()[0].shape
print(f"Input Shape: {input_shape}")

# Get the name of the first output of the model
output_name = sess.get_outputs()[0].name
print(f"Output Name: {output_name}")

# Print the output shape
output_shape = sess.get_outputs()[0].shape
print(f"Output Shape: {output_shape}")
