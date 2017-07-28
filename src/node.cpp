#include "node.h"


ReturnCode Conditional::tick()
{
	
	if (predicate())
	{
		cout << "predicate success" << endl;
		return ReturnCode::SUCCESS;
	}
	else
	{
		cout << "predicate failure" << endl;
		return ReturnCode::FAILURE;
	}
}

ReturnCode Action::tick()
{
	return action();
	
}

ReturnCode Sequence::tick()
{
	
	for (int i = currentChild; i < children.size(); ++i)
	{
		ReturnCode childstatus = children[i]->tick();

		currentChild = i;

		if (childstatus == ReturnCode::RUNNING)
		{
			return ReturnCode::RUNNING;
		}
		else if (childstatus == ReturnCode::FAILURE)
		{
			currentChild = 0;
			return ReturnCode::FAILURE;
		}
	}

	currentChild = 0;
	return ReturnCode::SUCCESS;

};

ReturnCode Selector::tick()
{
	
	for (int i = currentChild; i < children.size(); ++i)
	{
		ReturnCode childstatus = children[i]->tick();
		
		currentChild = i;

		if (childstatus == ReturnCode::RUNNING)
		{
			return ReturnCode::RUNNING;
		}
		else if (childstatus == ReturnCode::SUCCESS)
		{
			currentChild = 0;
			return ReturnCode::SUCCESS;
		}
		
	}
	currentChild = 0;
	
	return ReturnCode::FAILURE;

};