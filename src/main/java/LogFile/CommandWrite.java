package LogFile;

import java.util.LinkedList;

import edu.wpi.first.wpilibj.command.Command;

public abstract class CommandWrite extends Command {

	protected LinkedList<String> write;
	protected String commandName;
	protected boolean isExecute;

	public CommandWrite(String commadName, LinkedList<String> write) {
		this.commandName = commadName;
		this.write = write;
		this.isExecute = false;
	}

	protected abstract void initializeWrite();

	protected void initialize() {   	
		write.add(commandName + " is initialize");
		
		initializeWrite();

		this.isExecute = false;
	}

	protected abstract void executeWrite();

	protected void execute() {
		if(!this.isExecute){	
			write.add(commandName + " is execute");
			this.isExecute = true;
		}
		
		executeWrite();
	}

	protected abstract boolean isFinished();

	protected abstract void endWrite();

	protected void end() {
		write.add(commandName + " is end");
		this.isExecute = false;

		endWrite();
	}

	protected void interrupted() {
		write.add(commandName + " is interrupted");
		this.isExecute = false;

		endWrite();
	}
}
