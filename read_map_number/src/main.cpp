#include "read_map_number.h"

int main(int argc, char **argv)
{
	//string file_path = "/home/wzm/tmp/";
	string file_path = argv[1];
	string file_ext_name = "pb";
	HandleMapFile _handle(file_path, file_ext_name);
	std::cout << "start handle map file....." << std::endl;
	_handle.FindMapFile();

	for (int i = 0; i < _handle.mvMapFile.size(); i++)
	{
		cout << _handle.mvMapFile[i] << endl;
	}

	_handle.DeleteMapFile();

	_handle.SaveMapFileMaxIndex();

	std::cout << "msMapFileMaxIndex:" << _handle.msMapFileMaxIndex << std::endl;


	return 0;
}
