## ROBOCOM

robocom is the amalgamation of all important robot communcation nodes.

### command_execution_manager_2
is currently the active command execution manager. It receives all commands from the command_list_manager through the recv_cmd thread and forwards them to gripper, robot, camera client and axis.
right now it can freeze if the executed command breaks or any other errors happen.
### command_list_manager
keeps track of all the commands by storing it in a list. it woks on the recv_cmd and remove_frst_cmd threads to receive commands from various sources to append them to its list of commands (self.commandList)

### external_command_receiver
manages all communcation between external software by running a TCP server that receives commands to forward them to the command_list_manager. the current available format is with json compatible text.

### gripper_execution_note 
is a fork from https://gist.github.com/felixvd/d538cad3150e9cac28dae0a3132701cf#file-cmodel_urcap-py adapted to receive gripper commands and directly execute them

### queueshouter
shouts the queue via UPD for external software to read.

### robot_execution_node
executes robot commands through a combination oof PORT 30004 and PORT 30002


### rtde_state_publisher
takes PORT 30004 input to communicate the current state of the robot. needed by robot_execution_node

### manager_to_gripper_service, service_in_service_test
not sure if still needed
