#include "RobotModelUtil.h"
#include <iostream>
using namespace std;

void printAiMatrix4x4(aiMatrix4x4 m){
	cout << "Transformation: " << endl;
	cout << m.a1 << ", " << m.a2 << ", " << m.a3 << ", " << m.a4 << endl;
	cout << m.b1 << ", " << m.b2 << ", " << m.b3 << ", " << m.b4 << endl;
	cout << m.c1 << ", " << m.c2 << ", " << m.c3 << ", " << m.c4 << endl;
	cout << m.d1 << ", " << m.d2 << ", " << m.d3 << ", " << m.d4 << endl;
}

void printNode(const aiNode* node)
{
	cout << "Name: " << node->mName.data << endl;
	cout << "Parent: ";
	if(node->mParent){
		cout << node->mParent->mName.data << endl;
	}
	cout << "NumChildren: " << node->mNumChildren << endl;
	for(int i=0; i<node->mNumChildren; i++){
		cout << "Child: " << node->mChildren[i]->mName.data << endl;
	}
	cout << "NumMeshes: " << node->mNumMeshes << endl;
	printAiMatrix4x4(node->mTransformation);

	for(int i=0; i<node->mNumMeshes; i++){
		cout << "Meshes[" << i << "]: " << node->mMeshes[i] << endl;
	}
	cout << endl;
}

void searchPrintNode(const aiNode* node)
{	
	printNode(node);
	for(int i=0; i<node->mNumChildren; i++){
		searchPrintNode(node->mChildren[i]);
	}
}

void searchRegistOrigin(aiNode* node, map<string, aiMatrix4x4>& it)
{
	it[node->mName.data] = node->mTransformation;
	for(int i=0; i<node->mNumChildren; i++){
		searchRegistOrigin(node->mChildren[i],it);
	}
}