#ifndef __MODEL_H__
#define __MODEL_H__

#include <map_types.h>
#include <maps.h>
#include <time_prediction.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <exception>
#include <cstdio>
#include <memory>

#ifdef _MSC_VER
static const char* PATH_SEP = "\\";
#else
static const char* PATH_SEP = "/";
#endif

/*!
This class implements logic of extract maps from source files and bring access data.

*/
struct MapsModel
{
	static const size_t IMAGE_DIM; // Width and height of the elevation and overrides image

	MapsModel(const std::string& pname)
	{
		//Initialization. 
		const size_t expectedFileSize = getSizeX() * getSizeY();
		// Address assets relative to application location
		std::string anchor = std::string(".") + PATH_SEP;
		auto lastpos = pname.find_last_of("/\\");
		if (lastpos != std::string::npos)
		{
			anchor = pname.substr(0, lastpos) + PATH_SEP;
		}
		//Create maps
		m_elevation.reset(new  MapExplorer(
			loadFile(anchor + "assets" + PATH_SEP + "elevation.data", expectedFileSize),
			getSizeX(), getSizeY()));

		m_overrides.reset(new  MapExplorer(
			loadFile(anchor + "assets" + PATH_SEP + "overrides.data", expectedFileSize),
			getSizeX(), getSizeY()));
	}

	size_t getSizeX() const { return IMAGE_DIM; };

	size_t getSizeY() const { return IMAGE_DIM; };

	const MapExplorer& elevation() const { return *m_elevation.get(); }

	MapExplorer& elevation() { return *m_elevation.get(); }


	const MapExplorer& overrides() const { return *m_overrides.get(); }
private:
	std::ifstream::pos_type fileSize(const std::string& filename)
	{
		//TODO what wrong with s.t. like stat
		std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
		if (!in.good())
		{
			throw std::exception("Can't open file for get size");
		}
		return in.tellg();
	}

	std::vector<uint8_t> loadFile(const std::string& filename, size_t expectedFileSize)
	{
		size_t fsize = fileSize(filename);
		if (fsize != expectedFileSize)
		{
			throw std::exception("wrong file size");
		}
		std::vector<uint8_t> data(fsize);
		std::ifstream ifile(filename, std::ifstream::binary);
		if (!ifile.good())
		{
			throw std::exception("Can't open file");
		}
		ifile.read((char*)&data[0], fsize);
		return data;
	}

private:
	std::unique_ptr<MapExplorer> m_elevation;
	std::unique_ptr<MapExplorer> m_overrides;

};

const size_t MapsModel::IMAGE_DIM = 2048;
#endif // __MODEL_H__