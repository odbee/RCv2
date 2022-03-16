Msgs stores all the message types needed to communacate between each other.

the current messages are

- MoveGripper.srv Args: command, position, speed, force Return: returncode (int), returnpos (int), objectstatus (int)
- MoveRobot.srv send move robot command. Args: string, Return: int
- ReceiveCommand.srv
- RequestOCV.srv: sends  request for openCV command/data
- SendORderReturnResult.srv: sends oder as string and return the result
- FollowGripper.action: send commands to move the gripper and upate its current gap size
