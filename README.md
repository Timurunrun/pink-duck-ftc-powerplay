# pink-duck-ftc-powerplay
That's the code for Pink Duck Teams robot for the FTC POWERPLAY championship season.
## How to figure out what's what
The repository contains two folders, Autonomus and Teleop. As the folder «TeleOp» contains only one file (actually teleop), we will therefore take apart the folder «Autonomus».

In order to select the required programme, you should:
1. Select the colour of the robot’s installation side
2. Find out the autonomous period strategies of the other teams involved in the match
All programs for the blue side contain «Blue» in their name, respectively the programs for the red side contain «Red» in their name.

Two files, «AutoMethods» and «Detector», are universal for all programmes in the autonomous period.
The file «AutoMethods» contains the methods used in the autonomous period (e.g. motions).
The «Detector» file contains code to determine the side of the signal sleeve (for further parking at the end of the autonomous period). Since the sides of our signal sleeve are blue, yellow and bands of yellow and blue, the «Detector» filters the content of yellow in a certain area of the camera image, and based on its quantity decides which side the signal sleeve is now installed to the robot).
## How to use it
### Installation
Download the repository and move it to your «Teamcode» folder (unzip the folders and delete the file «readme.md»).
### Run
Enter this at the command line of your IDE:
```
cd [YOUR PATH TO "android\sdk\platform-tools" (we have C:\Users\user\AppData\Local\Android\Sdk\platform-tools)] 
.\adb connect [IP ADDRESS OF YOUR ROBOT (we have 192.168.43.1)]
```
If you are already connected but are experiencing problems specifically with programme transfer, reconnect using this code:
```
.\adb disconnect
.\adb connect [IP ADDRESS OF YOUR ROBOT (we have 192.168.43.1)]
```
## Connect with Pink Duck Teams
You can contact us via our official [VK group](https://vk.com/pink.duck.teams).
