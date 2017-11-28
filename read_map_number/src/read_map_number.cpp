#include "read_map_number.h"

#define linux

#ifdef linux  
#include <unistd.h>  
#include <dirent.h>  
#endif  
#ifdef WIN32  
#include <direct.h>  
#include <io.h>  
#endif 

bool HandleMapFile::FindMapFile()
{
#ifdef WIN32
	string map_file_path = msMapFilePath+"*";  
	_finddata_t file;
	long lf;
	if ((lf = _findfirst(map_file_path.c_str(), &file)) == -1) {
		cout << msMapFilePath << " not found!!!" << endl;
		return false;
	}
	else {
		while (_findnext(lf, &file) == 0) {
			if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
				continue;

			if (IsMapFile(file.name))
			{
				mvMapFile.push_back(file.name);
				mMapFileCount++;
			}
			else
			{
				std::cout << "not map file:" << file.name << std::endl;
				continue;
			}
		}
	}
	_findclose(lf);
#endif  

#ifdef linux  
	DIR *dir;
	struct dirent *ptr;
	std::cout << "msMapFilePath:" << msMapFilePath << std::endl;	

	if ((dir = opendir(msMapFilePath.c_str())) == NULL)
	{
		perror("Open dir error...");
		return false;
	}

	while ((ptr = readdir(dir)) != NULL)
	{
		if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) 
			continue;
		else if (ptr->d_type == 8)
		{
			if (IsMapFile(ptr->d_name))
			{
				mMapFileCount++;
				mvMapFile.push_back(ptr->d_name);
			}
			else
			{
				std::cout << "not map file:" << ptr->d_name << std::endl;
				continue;
			}
		}
		else if (ptr->d_type == 10)   
			continue;
		else if (ptr->d_type == 4)  
		{
			continue;
		}
	}
	closedir(dir);
#endif  
	SortMapFile(mvMapFile);
	return true;
}
std::vector<std::string> HandleMapFile::MapFileSplit(const  std::string _s, const std::string _delim)
{
	//split string according to '.'
	std::vector<std::string> elems;
	size_t pos = 0;
	size_t len = _s.length();
	size_t delim_len = _delim.length();
	if (delim_len == 0) return elems;
	while (pos < len)
	{
		int find_pos = _s.find(_delim, pos);
		if (find_pos < 0)
		{
			elems.push_back(_s.substr(pos, len - pos));
			break;
		}
		elems.push_back(_s.substr(pos, find_pos - pos));
		pos = find_pos + delim_len;
	}
	return elems;
}

bool HandleMapFile::SortMapFile(string map_a,string map_b)
{
	//according to file ext name and sort map file
	std::vector<std::string> map_file_temp_a = MapFileSplit(map_a, ".");
	std::vector<std::string> map_file_temp_b = MapFileSplit(map_b, ".");

	istringstream convert_a(map_file_temp_a[0]);
	istringstream convert_b(map_file_temp_b[0]);

	int map_file_int_a, map_file_int_b;

	if (!(convert_a >> map_file_int_a))
	{
		map_file_int_a = 0;
	}

	if (!(convert_b >> map_file_int_b))
	{
		map_file_int_b = 0;
	}
	
	std::cout << "map_file_int_a:" << map_file_int_a << std::endl;
	std::cout << "map_file_int_b:" << map_file_int_b << std::endl;
	return map_file_int_a < map_file_int_b;

}
void HandleMapFile::SortMapFile(vector<string>& _mvMapFile)
{
	//according to file ext name and sort map file
	for (int i = 0; i < _mvMapFile.size(); i++){
		for (int j = 0; j < _mvMapFile.size() - i - 1; j++)
		{
			if (!SortMapFile(_mvMapFile[j], _mvMapFile[j+1]))
			{
				string temp = _mvMapFile[j];
				_mvMapFile[j] = _mvMapFile[j+1];
				_mvMapFile[j+1] = temp;
			}
		}
	}
}
bool HandleMapFile::IsMapFile(string map_file)
{
	//checkout file ext name
	std::vector<std::string> map_file_temp = MapFileSplit(map_file, ".");
	
	assert(map_file_temp.size() >=1 );

	if (map_file_temp.size() == 2)
	{
		std::cout << map_file_temp[0] << endl;
		std::cout << map_file_temp[1] << endl;
		if (map_file_temp[1] == msMapFileExtName)
			return true;
		else
			return false;
	}
	else
		return false;
}
void HandleMapFile::DeleteMapFile()
{
	
	if (mMapFileCount > MAP_LIMIT_N)
	{
		for (int i = 0; i < mMapFileCount - MAP_LIMIT_N; i++)
		{
			string map_file_deleted = msMapFilePath + mvMapFile[i];
			std::cout << "map_file_deleted:" << std::endl;
			std::cout << map_file_deleted << std::endl;
			remove(map_file_deleted.c_str());
		}
	}
}
void HandleMapFile::SaveMapFileMaxIndex()
{
	if (mMapFileCount > 0)
	{
		std::vector<std::string> map_file_temp = MapFileSplit(mvMapFile[mMapFileCount - 1], ".");
		istringstream convert(map_file_temp[0]);
		int map_file_max_index;

		if (!(convert >> map_file_max_index))
		{
			map_file_max_index = 0;
		}

		std::cout << "map_file_max_index:" << map_file_max_index << std::endl;
		stringstream ss;
		ss << map_file_max_index;
		msMapFileMaxIndex = ss.str();

		//msMapFileMaxIndex = to_string(map_file_max_index);
		std::cout << "msMapFileMaxIndex:" << msMapFileMaxIndex << endl;

		//mFilePtr.write(ss.str().c_str(), ss.str().length());
	}
	else
	{
		stringstream ss;
		ss << 1;
		msMapFileMaxIndex = ss.str();
		std::cout << "msMapFileMaxIndex:" << msMapFileMaxIndex << endl;
	}
		

}
