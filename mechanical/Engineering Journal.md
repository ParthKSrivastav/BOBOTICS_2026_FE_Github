This section shows how the robot changed from 2024 to 2026, what technical choices we made along the way, and what we learned from each stage. We did not want to just show the final robot, we wanted to show the full process, including the failures, redesigns, and improvements that led to the finished version.
2024: Early experiments

We started in 2024 by trying out two SPIKE hubs connected together so we could get twelve ports in total. We flashed both hubs to Robot Inventor because the platforms are based on the same system. In our minds, this looked like a good idea because it gave us more room for motors and sensors, but in reality it was too slow and too inaccurate for a WRO robot.

During testing, we noticed that the dual-hub setup caused delays and inconsistent behavior. Since WRO Future Engineers needs a robot that is fast, stable, and repeatable under pressure, we decided that this setup was not good enough and moved on.

After that, we cut the second hub out and tried using just one SPIKE hub. At this point, we also tried to make the robot look more polished, like our later Mark 5 version. However, we soon realised that the chassis was too heavy, and that made turning much worse.

So we removed extra weight and simplified the design into Mark 6. That fixed the weight issue, but it also showed us another problem: the SPIKE motor itself was not accurate enough for the level of turning precision we needed.
Which led us to move to Raspberry Pi

Because of the limitations on SPIKE, we decided to switch to a Raspberry Pi, we choice the 4B with 8 GB of RAM. We chose it because it was beginner-friendly, affordable, yet powerful enough to run everything we needed without having to upgrade again later.

The move was not smooth at first. When the Raspberry Pi arrived, we had bought the wrong HDMI cable(HDMI mini), so we could not boot it in the recommended way. We used Raspberry PI imager to flash the SD card and pre-set Wi-Fi so it could connect remotely once it booted. Then used PuTTY to communicate using SSH(Secure Shell) 

We used a 64 GB SD card, which was more than enough for the project. Even after flashing it, though, the Pi still would not connect properly through PuTTY over SSH. We spent weeks troubleshooting it, re-flashing the SD card many times, until we finally got it working. The problem was that we had not enabled the SSH communication on the raspberry pi through the pi imager

Once SSH worked, we set up VNC too, which let us use the laptop keyboard, mouse, and screen remotely with the Pi. This part did not directly make the robot better, but it gave us the computing platform we needed for later camera work, sensor integration, and more advanced software.
2025: Stable Round 1, but too much change

In 2025, we did not spend as much time on the robot overall, but we did get the Open Challenge working consistently. That was the first time the robot became genuinely repeatable in Round 1.

Every time we got it to work, we kept trying to improve it by adding more sensors. The problem was that every new addition seemed to break something that already worked. That taught us an important lesson: close to competition, untested hardware or software changes can make a reliable robot worse, not better and if it works do not touch.

From that point on, we set ourselves one rule: if competition is less than four months away, we should avoid major untested changes unless they are really necessary. We also learned that working code must always be backed up in more than one place, because stable code should never be risked carelessly.

In 2025, we also qualified for the international finals. However, only Parth was old enough to compete, while Raphael was too young. Because we could not enter with only one member and could not find anyone willing to pay and join, we had to withdraw. That was frustrating, but it also showed us how important planning, eligibility, and team readiness are, not just robot performance.
2026: We had less time but we were more focused

In 2026, we had less time to work on the robot because we were in Year 10 and had mocks and GCSEs to prepare for. That meant we had to reduce the time for the robot and prioritize studies, this hindered progress 
January

In January, our first set of mocks meant we had no time to work on the robot together. Development basically paused for that month.
February

In February, we got back to work and improved the camera system so it could detect red and green objects using our own colour-detection logic. We used OpenCV to process the images and Picamera2 to give us the camera feed.

This was a big step for the Obstacle Challenge, because obstacle recognition is one of the key parts of that round. Instead of using a ready-made system, we built our own colour-based detection logic for the traffic markers.
March

In March, we tried to improve turning accuracy by working on the gyro. The goal was to use orientation feedback so the robot could turn more cleanly and consistently at corners.
April and May

In April, we did not work on the robot because our first big exam was coming up in May. In May, GCSEs started, so again we had very limited time for robot development.
June

In June, we removed the gyro from the main turning system for the time being. At that point, motor-controller timing was giving us more repeatable results than the gyro setup we were testing, so we simplified the code and used the controller timer instead.
August was our biggest month, we changed the whole robot to a new smaller design taking inspirational from a Japanese kei-truck, we also changed the whole code structure to a ROS2 system which provided more consistency but also added a more complexity to the robot with the need of work around like in the camera. We also completed the full mission with parking and made the 3 laps even more accurate.
This was not the perfect solution, but it was the most reliable one for that stage of the project. We did not give up on the gyro completely — it just was not the best option at that moment.
In august we have overhauled our old design and went for a new structure. We installed docker and ROS onto our debian(raspberry pi software) to have a more accurate and consistent robot. We decided that our old code was getting too complicated and was going to hard to debug in the pressure of the competition. So we invested time into ROS2 which acts like a nervous system for the robot and helps us to find problems more easily (if a sensor was not working properly we can test if it is the sensor or if it s the node which can fast track debugging). Although this does go against our learning of not touching something that works and we did have thoughts this will be risky and can cause has to lose precious time but we chose it is worth the risk and went on.
What does ROS2 actually do - it acts as a system that takes the info from the sensors and gives it to one "main" code in our case is navigation node. ROS does not do anything more than make the system, it is not a type of intelligence as we first mistaken it for. However, it is still worth if your code seems complicated.
We also added a BNo085 gyro which has caused us problems which we talk about in https://github.com/ParthKSrivastav/RoboLords_2026_FE_Github/blob/main/code/README.md and how we solved the issue.
Competition lessons in 2026
Regionals

At regionals, we won, but we had one major issue: the robot would not connect properly to the laptop. That was a serious risk, because it showed that our setup still depended too much on network-based access.

To prevent that happening again, we started bringing extra equipment to competitions, including a monitor, mouse, and keyboard. We also connected a physical button-based startup system so the robot could be started safely and edited locally without needing internet access.
This did not make the robot faster, but it made the whole system much more reliable and competition-ready.

Finals
At finals, we won again, but we also came very close to being disqualified because our truck had one more axle than the rules allowed. It was originally based on a six-wheeled RC truck that we had tuned heavily.
To stay within the rules, we replaced it with the smaller mini-truck design used in Mark 11 and Mark 12. That taught us an important lesson: even when a robot performs well, it still has to stay legal.
August: Final improvements
During the summer holidays in August, we focused hard on both rounds. We made the Open Challenge much more accurate, and we improved the camera logic so it could react properly to both red and green obstacles.
At that stage, we added a BNO085 gyro to make 45-degree turns more accurate in Round 2, especially for the Obstacle Challenge. We also added parking logic and tuned the code further so the robot would be more repeatable overall.
We also tested different speeds to determine which speed gave the most accuracy, 0.5(50%) led to the robot to finish in 20 - 25 seconds, which would have given us top o10 in round 1 however it had a 5/10 chance of completing, so 50% chance to fail which was too high for us. So we lowered the speed down to 0.3 and realised it did not improve the accuracy but made the robot take 23-28 seconds and only complete 7/10. So we lowered it to 0.2 which gave us 9/10 accuracy and made it take 25-30 seconds which is worth more to us than having no points.
By then, the biggest lessons from the last two years were clear: avoid unnecessary complexity, use vision only when it really helps, focus on repeatability instead of just raw speed and most importantly, do not touch what works
What we learned:
A few main lessons came out of the whole process:
 -> More hardware is not always better. Extra hubs and sensors can add complexity without improving performance.
 -> Looks should never come before function. Heavy or stylish designs can hurt turning and control.
 -> A consistent robot is more valuable than a fast but unreliable one.
 -> Untested changes close to competition are risky.
 -> Having a reliable local startup system matters a lot in real competition conditions.
 -> Rule compliance has to be checked carefully every time the design changes.
