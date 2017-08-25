#pragma once

#ifdef CAVESEGMENTATIONLIB_EXPORT
#define CAVESEGMENTATIONLIB_API __declspec(dllexport)
#else
#define CAVESEGMENTATIONLIB_API __declspec(dllimport)
#endif