#ifndef READ_MAP_NUMBER_H
#define READ_MAP_NUMBER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <assert.h>
#include <vector>
#include <stdlib.h>
#include <algorithm> 
#include <functional>
#include <stdio.h> 
#include <string.h> 


using namespace std;

class HandleMapFile
{
public:
	HandleMapFile(string _msMapFilePath, string _msMapFileExtName)
	{
		msMapFilePath = _msMapFilePath;
		msMapFileExtName = _msMapFileExtName;
		mMapFileCount = 0;
		//mFilePtr.open(msMapIndexFile.c_str(), ios::in | ios::out|ios::binary);
	}
	~HandleMapFile()
	{
		//mFilePtr.clear();
		//mFilePtr.close();
	}
	bool FindMapFile();
	void DeleteMapFile();
	bool SortMapFile(string map_a, string map_b);
	void SortMapFile(vector<string>& _mvMapFile);
	bool IsMapFile(string map_file);
	std::vector<std::string> MapFileSplit(const  std::string _s, const std::string _delim);
	void SaveMapFileMaxIndex();


	string msMapFilePath;//map path 
	string msMapFileExtName;//map extension name
	string msMapFileMaxIndex;//max map index

	vector<string> mvMapFile;

	int mMapFileCount;

	//fstream mFilePtr;
private:
	static const int MAP_LIMIT_N = 8;
};
#endif
