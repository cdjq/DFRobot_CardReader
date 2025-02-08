import time
from rc522 import Rfid
def main():
  rfid_reader = Rfid()
  rfid_reader.begin()
  while True:
    print("scaning...")
    if rfid_reader.scan():
      print(f"success RFID ，uuid: {rfid_reader.read_uid()}")
      write_data = [1,3,1,2,3,4,5,6,7,8,9,11,12,13,14,15]
      print(f"write data addr 5: {write_data}")
      rfid_reader.write_block(5,write_data)
      time.sleep(2)
      print(f"read data addr 5:{rfid_reader.read_block(5)}")
      
      rfid_reader.write_block(5,0xDF,1)
      time.sleep(2)
      print(f"read data addr 5:{rfid_reader.read_block(5,1)}")
    time.sleep(1) 
if __name__ == "__main__":
    main()