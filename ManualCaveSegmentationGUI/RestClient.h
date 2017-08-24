#pragma once

#include <thread>
#include "CaveData.h"
#include <QFile>

#include <cpprest/http_client.h>
#include <cpprest/filestream.h>
using namespace utility;                    // Common utilities like string conversions
using namespace web;                        // Common features like URIs.
using namespace web::http;                  // Common HTTP functionality
using namespace web::http::client;          // HTTP client features
using namespace concurrency::streams;       // Asynchronous streams

class NetworkException : public std::exception
{
public:
	NetworkException(int code) : code(code) {  }

	std::string what() 
	{
		std::stringstream ss;
		ss << "Network error. Response code " << code;
		return ss.str();
	}
	
	int code;
};


class RestClient
{
private:
	std::unique_ptr<http_client> client;

	pplx::task<web::json::value> GetJSON(const std::wstring& path);
public:


	RestClient(std::wstring serviceUrl);

	pplx::task<std::shared_ptr<std::vector<CaveMetadata>>> GetCaves();

	pplx::task<void> DownloadCave(int caveId, const QString& targetFile);

	pplx::task<void> UploadSegmentation(int caveId, const std::wstring segmentationB64, const std::wstring uploaderName, int expertise, int certainty);
};

template <typename T>
struct TaskResult
{
	bool success;
	T result;
};

template <>
struct TaskResult < void >
{
	bool success;
};

template <typename T>
pplx::task<TaskResult<T>> GetTaskResult(pplx::task<T> t)
{
	return t.then([=](pplx::task<T> t)
	{
		TaskResult<T> result;
		try
		{
			result.result = t.get();
			result.success = true;
		}
		catch (...)
		{
			result.success = false;
		}
		return result;
	});
}


template <>
extern pplx::task<TaskResult<void>> GetTaskResult(pplx::task<void> t);