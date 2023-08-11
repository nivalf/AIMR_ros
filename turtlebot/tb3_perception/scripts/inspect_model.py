import torch

model_path = "../models/assembly_system_detector.pt"
model = torch.load(model_path)
model.eval() # Sets the model to evaluation mode

print(model)

dummy_input = torch.randn(1, 3, 320, 240) # Example input shape
with torch.no_grad():
    dummy_output = model(dummy_input)

print("Input shape:", dummy_input.shape)
print("Output shape:", dummy_output.shape)

for name, param in model.named_parameters():
    print(name, param.size())
