Group3: Automatic Seed Sowing Project
-------------------------------------------

Team:
----------
Deepak Bhat     	113079007 
Newton                  113079018 
Ashwin Radhakrishnan    113079011 

Hardware requirements:
-------------------------
? Wheel encoder 
? Zigbee USB adaptor 
? Zigbee transmitter and receiver modules 
? Onboard battery support 
? Sharp sensors 
? Servomotor 
? Fire-bird V 

Steps to run the project
1. Open the project "alignment.aps" in AVR studio.
2. Build the project to generate alignment.hex file
3. Load the hex file into the Firebird V controller using AVR Bootloader.
4. Enter data in the following format in the remote GUI
	"2(BOTID)$1(TASKID)$0to7(TroughID)#"
   to set the robot moving for seed sowing
   
 
Steps to run MATLAB code for offline verification of seed sowing process
1. Open the inter.m code in MATLAB.
2. Save image of the seeds sown on a trough as "image.jpg"  and place
   it in the same folder as that of MATLAB code
3. Run the matlab code to generate the processed image, mean and variance details
   
   