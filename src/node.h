#ifndef NODE_H
#define NODE_H

/*#include "behaviortree.h"*/
#include "returncode.h"
#include <vector>
#include <functional>
#include <iostream>

using namespace std;


class Node
{


public:
	Node(){};
	~Node(){};

	virtual ReturnCode tick(){ };
	
};


class Leaf : public Node
{

};

class Action : public Leaf
{
private:
	// http://www.learncpp.com/cpp-tutorial/78-function-pointers/
	function<ReturnCode()> action;

public:
	Action(){};
	Action(function<ReturnCode()> act){ action = act; };
	~Action(){};
	
	ReturnCode tick();

};


class Conditional : public Leaf
{
private:
	//http://www.learncpp.com/cpp-tutorial/78-function-pointers/
	function<bool()> predicate;

public:
	Conditional(){};
	Conditional(function<bool()> pred){ predicate = pred; };
	~Conditional(){};
	ReturnCode tick();

};




class Composite : public Node
{
protected:
	int currentChild;
	vector<Node*> children;

public:
	Composite(){ currentChild = 0; };
	~Composite(){};
	
	void addChild(Node *n) { children.push_back(n); }
	virtual ReturnCode tick() {};
};

class Sequence : public Composite
{
public:
	Sequence(){};
	~Sequence(){};
	ReturnCode tick();
};

class Selector : public Composite
{
public:
	Selector(){};
	~Selector(){};
	ReturnCode tick();
};


#endif // NODE_H
