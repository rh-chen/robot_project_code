#include "read_map_number.h"

int main(int argc, char **argv)
{
	string file_path = argv[1];
	string file_ext_name = "pb";
	HandleMapFile _handle(file_path, file_ext_name);
	_handle.FindMapFile();//find map file and sort

	for (int i = 0; i < _handle.mvMapFile.size(); i++)
	{
		cout << _handle.mvMapFile[i] << endl;
	}

	_handle.DeleteMapFile();//delete map file

	_handle.SaveMapFileMaxIndex();//get max map file name

	std::cout << "msMapFileMaxIndex:" << _handle.msMapFileMaxIndex << std::endl;


	return 0;
}
