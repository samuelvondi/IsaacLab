import hid

for device in hid.enumerate():
    print(f"Product String: {device.get('product_string')}")
    print(f"Vendor ID: {hex(device['vendor_id'])}")
    print(f"Product ID: {hex(device['product_id'])}")
    print(f"Manufacturer: {device.get('manufacturer_string')}")
    print(f"Path: {device.get('path')}")
    print("-" * 40)
