import yaml

stream = open("./src/data/known_peripherals_copy.yaml", "r")
print(yaml.load(stream))