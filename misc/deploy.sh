#!/bin/bash

OPTION=$1

roverpass=""
roverIP=""
branch=""
needsReboot=false

cd ..
dirPath="$(pwd)"
dirName="$(basename `pwd`)"

#Functions
#--------------------------------------------------------------------------

Check()
{
	cd ~

	if [ -z "$(ls -A $dirPath/src/ublox/)" ]; then
  		echo "The ublox submodule is missing."
  		exit 1
	fi

	if [ -z "$(ls -A $dirPath/src/apriltags_ros/)" ]; then
  		echo "The apriltags submodule is missing."
  		exit 1
	fi
}

PullGit_Pack()
{
	info="Pulling from $branch, Compiling, and Packing..."
	gnome-terminal -x bash -c "echo -n -e '\033]0;$info\007';
		cd $dirPath;
		echo 'Checking out branch $branch';
		git checkout $branch &&
		{
			echo 'Pulling latest build on branch $branch';
			git pull && 
			{
				echo 'Compiling $dirName branch $branch...';
				catkin build &&
				{
					cd -;
					echo 'Packing up the repository... ';
					tar czf $dirName.tgz $dirPath;
					exit 1;
				}
				read stuff;
				exit 1; 
			}
			read stuff;
			exit 1;
		}		
		read stuff
		exit 1;
		/bin/bash;' exec $SHELL"
}

Pack()
{
	info="Compiling and Packing Local Files..."
	gnome-terminal --disable-factory -x bash -c "echo -n -e '\033]0;$info\007';
		cd $dirPath;
		{
			echo 'Compiling $dirName';
			catkin build &&
			{ 
				cd ~;
				echo 'Packing up the repository... ';
				tar czf $dirName.tgz $dirName;
				exit 0;
			}
			read stuff
			exit 1;
		}
		/bin/bash;' exec $SHELL"
}

Transfer()
{
	echo "Copying compressed repository to swarmie at $roverIP"
	scp $dirName.tgz swarmie@$roverIP:~
}

Unpack_Run()
{
	echo "Unpacking repository $dirName on swarmie at $roverIP"

	gnome-terminal --tab -x bash -c "echo -n -e '\033]0;$roverIP\007'
		ssh -t swarmie@$roverIP 'echo 'Unpacking $dirName.tgz ...';
		tar xzf $dirName.tgz;
		sleep 2;			
		echo 'Starting ROS nodes on swarmie at $roverIP with master at $hostName';
		sleep 2;
		cd $dirName/misc/;
		./rover_onboard_node_launch.sh $hostName;
		exit 1;
		/bin/bash;' exec $SHELL"
		
		echo "Starting ROS nodes on swarmie at $3 with master at $hostName"
		echo ""
}

Run()
{
	#ssh and run script from rover --WORKS
	gnome-terminal --tab -x bash -c "echo -n -e '\033]0;$roverIP\007';
		ssh -t swarmie@$roverIP 'echo 'Running $roverIP';
		cd SwarmBaseCode-ROS/misc;
		./rover_onboard_node_launch.sh $hostName;
		exit 1;
		exit 1;
		/bin/bash;' exec $SHELL"
}

DispOpt()
{
	echo "Your network info:  $hostName@$userIP"
	echo ""
	echo "Type 'exit' to quit"

	if [ "$OPTION" != "-G" ]; then
		echo "Type '-G' to pull, transfer, and run on swarmie(s) "
	elif [ $OPTION == "-G" ]; then
		echo "Type 'RP' to Re Pull from the github Repository"
		echo "Type 'NB' to pull from a new Branch"
	fi

	if [ "$OPTION" != "-R" ]; then
		echo "Type '-R' to run current code loaded on Swarmies"
	fi
	
	if [ "$OPTION" != "-L" ]; then
		echo "Type '-L' to transfer local code and run on swarmie(s)"
	elif [ $OPTION == "-L" ]; then
		echo "Type 'RC' to recompile Source Code"
	fi

	echo "Type reboot {hostname} to reboot a swarmie"

	echo "-------------------------------------------------------------"
	echo ""
}

SuccessPing()
{
	cd ~
	echo "Found $roverIP"
	echo "-------------------------------------------------------------"
}

FailPing()
{
	echo "$roverIP was not found on the network.  Check IP and network settings!"
	echo "-------------------------------------------------------------"	
	sleep 4
	echo ""
}

Reboot()
{
	echo "REBOOTING"
	info="Rebooting $roverIP"
	gnome-terminal --tab -x bash -c "echo -n -e '\033]0;$info\007';
		ssh -t swarmie@$roverIP 'sudo reboot now';
		exit 1;
		/bin/bash;' exec $SHELL"
}

WaitForReconnect()
{
	while(true); do
		ping -q -w2 $roverIP > /dev/null;
		if [ $? -eq 0 ]; then
			echo "$roverIP is up"
			break
		fi
	done
}

#Code Start
#-----------------------------------------------------------------

#find users current IP and store it
userIP=$(ifconfig  | grep 'inet addr:'| grep -v '127.0.0.1' | cut -d: -f2 | awk '{ print $1 }')
hostName=$(hostname)

echo "$dirPath"
echo "$dirName"

#No Connection
#-------------------------------------------------------------
if [ -z "$userIP" ]; then
	echo "No Current Internet Connection Found"
	echo "Cannot SSH into rover without IP Address!"
	exit 1
fi

#No Command Given
#-------------------------------------------------------------
if [ -z $OPTION ]; then
	echo "-------------------------------------------------------------"
	echo "Please run this script with the following commands:"
	echo "'-G'[ithub] pulls and transfers workspace to swarmie(s)"
	echo "'-L'[ocal Files] duplicates current workspace and transfers to swarmie(s)"
	echo "'-R'[un] to run current code on all swarmie(s)"
	echo "-------------------------------------------------------------"
	exit 1
fi

clear

#check OPTION given
#------------------------------------------------------------

while(true); do

#GitHub Selection
#------------------------------------------------------------
if [ $OPTION == "-G" ]; then

	branch=$2

	#if not everything is filled out
	if [ "$branch" == "" ]; then
		echo ""
    		read -p "Branch?:  " branch
		echo ""
	fi

	#Tell User Option

	echo "Retrieving and Transferring Code from GitHub:  "

	echo "dir: $dirPath branch: $branch"
	echo "-------------------------------------------------------------"
	echo ""

	Check

	DispOpt

	PullGit_Pack &
	wait

	while(true); do
		read -p "Rover Name/IP To Start:  " roverIP add
			if [ "$roverIP" == "exit" ]; then
				exit 1
			elif [ "$roverIP" == "-R" ]; then
				OPTION="-R"
				clear
				break	
			elif [ "$roverIP" == "-L" ]; then
				OPTION="-L"
				clear
				break
			elif [ "$roverIP" == "NB" ]; then
				branch=""
				clear
				break
			elif [ $roverIP == "RP" ]; then
				clear
				echo ""
				echo "-------------------------------------------------------------"
				echo ""
				echo "Pulling new code from $branch"
				echo ""
			elif [ $roverIP == "reboot" ]; then
				roverIP=$add
				echo "Rebooting $roverIP"
				needsReboot=true				
			fi

		#If rover is on the network
		ping -q -w2 $roverIP > /dev/null
		if [ $? -eq 0 ]; then

			SuccessPing

			if [ $needsReboot == true ]; then
				Reboot &
				wait
				WaitForReconnect &
				wait
			fi

			#Transfer/Unpack/Run
			Transfer &
			wait
			Unpack_Run &
			wait			

		#if not on the network
		else
			FailPing
		fi

	needsReboot=false

	done

fi

#Transfer Local Copy
#-------------------------------------------------------------
if [ $OPTION == "-L" ]; then

	Check

	DispOpt

	echo "Copying and transferring current folder to swarmie(s)"

	echo "dir: $dirPath"
	echo "-------------------------------------------------------------"
	echo ""

	Pack &
	wait

	while [ true ]; do
		read -p "Rover Name/IP To Start:  " roverIP add
			if [ $roverIP =  "exit" ]; then
				exit 1
			elif [ $roverIP = "-G" ]; then
				OPTION="-G"
				clear
				break	
			elif [ $roverIP = "-R" ]; then
				OPTION="-R"
				clear
				break
			elif [ $roverIP = "RC" ]; then
				clear
				echo ""
				echo "-------------------------------------------------------------"
				echo ""
				echo "Recompiling Source Code!"
				echo ""
				break
			elif [ $roverIP == "reboot" ]; then
				roverIP=$add
				echo "Rebooting $roverIP"
				needsReboot=true				
			fi

			#If rover is on the network
			ping -q -w2 $roverIP > /dev/null
			if [ $? -eq 0 ]; then

				SuccessPing

				if [ $needsReboot == true ]; then
					Reboot &
					wait
					WaitForReconnect &
					wait
				fi

				#Transfer/Unpack/Run
				Transfer &
				wait
				Unpack_Run &
				wait

			#if not on the network
			else
				FailPing
			fi

			needsReboot=false
	done
fi

#Just Run
#-------------------------------------------------------------
if [ $OPTION == "-R" ]; then

	DispOpt

	echo "Running current swarmie(s) code"
	echo "-------------------------------------------------------------"
	echo ""

	while(true); do
		read -p "Rover Name/IP To Start:  " roverIP add
			if [ "$roverIP" =  "exit" ]; then
				exit 1
			elif [ "$roverIP" = "-G" ]; then
				OPTION="-G"
				clear
				break	
			elif [ "$roverIP" = "-L" ]; then
				OPTION="-L"
				clear
				break
			elif [ $roverIP == "reboot" ]; then
				roverIP=$add
				echo "Rebooting $roverIP"
				needsReboot=true				
			fi

		#If rover is on the network
		ping -q -w2 $roverIP > /dev/null
		if [ $? -eq 0 ]; then

			SuccessPing

			if [ $needsReboot == true ]; then

				Reboot &
				wait

				WaitForReconnect &
				wait

			fi

			Run &
			wait

		#if not on the network
		else	
			FailPing
		fi

		needsReboot=false

	done
fi

done
