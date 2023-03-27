import sounddevice as sd
input_dev_num = sd.query_hostapis()[0]['default_input_device']
device_info = sd.query_devices(input_dev_num, 'input')
print(device_info)
print(input_dev_num)