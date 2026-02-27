import csv

def crc_calculation(bitstream):
    working_data = list(bitstream + "000000000000000")
    poly = "100010110011001" 
    
    for i in range(len(bitstream)):
            if working_data[i] == '1':
                working_data[i] = '0' 
                for j in range(15):

                    if poly[j] == '1':
                        if working_data[i + 1 + j] == '1':
                            working_data[i + 1 + j] = '0'
                        else:
                            working_data[i + 1 + j] = '1'
    remainder_string = "".join(working_data[-15:])
    return int(remainder_string, 2)


def validate_can_frames(filename):
    
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        
        for row in reader:
            frame_id = int(row['id'], 16)
            ide = int(row['ide'])
            rtr = int(row['rtr'])
            dlc = int(row['dlc'])
            given_crc = int(row['crc'], 16)
            
            cleaned_string = row['data'].strip()
            if cleaned_string != "":
                data_bytes = row['data'].split()
            else:
                data_bytes = []
            

            if frame_id > 0x7FF:             
                print(f"{row['id']:<6} -> bad_id")
                continue
            if dlc > 8:                       
                print(f"{row['id']:<6} -> bad_dlc")
                continue
            if len(data_bytes) != dlc:        
                print(f"{row['id']:<6} -> mismatch_of_dlc_and_data_frame")
                continue
                

            sof_bit = "0"                     
            id_bits = f"{frame_id:011b}"      
            rtr_bit = str(rtr)                
            ide_bit = str(ide)                
            r0_bit = "0"                      
            dlc_bits = f"{dlc:04b}"           
            
            data_bits = ""
            for byte in data_bytes:
                data_bits += f"{int(byte, 16):08b}"                 
            bitstream = sof_bit + id_bits + rtr_bit + ide_bit + r0_bit + dlc_bits + data_bits
            
            
            calc_crc = crc_calculation(bitstream)
            
            
            if calc_crc == given_crc:
                print(f"{row['id']:<6} -> none")
            else:
                print(f"{row['id']:<6} -> bad_crc")

validate_can_frames('can_frames.csv')