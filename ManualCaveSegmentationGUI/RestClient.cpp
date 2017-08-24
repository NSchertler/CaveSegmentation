#include "RestClient.h"

template <>
pplx::task<TaskResult<void>> GetTaskResult(pplx::task<void> t)
{
	return t.then([=](pplx::task<void> t)
	{
		TaskResult<void> result;
		try
		{
			t.wait();
			result.success = true;
		}
		catch (...)
		{
			result.success = false;
		}
		return result;
	});
}

pplx::task<web::json::value> RestClient::GetJSON(const std::wstring& path)
{
	uri_builder builder(U("/api"));
	builder.append_path(path);
	auto url = builder.to_string();
	return client->request(methods::GET, builder.to_string())
		.then([=](http_response response)
	{
		// Handle response headers arriving.
		if (response.status_code() != 200)
			throw NetworkException(response.status_code());

		//Extract JSON
		return response.extract_json();
	});
}


RestClient::RestClient(std::wstring serviceUrl)
{
	client = std::make_unique<http_client>(serviceUrl);
}

pplx::task<std::shared_ptr<std::vector<CaveMetadata>>> RestClient::GetCaves()
{
	return GetJSON(L"Caves").then([=](web::json::value v)
	{
		auto result = std::make_shared<std::vector<CaveMetadata>>();

		auto arr = v.as_array();
		for (auto entry : arr)
		{
			CaveMetadata data;
			data.Id = entry.at(L"Id").as_number().to_int32();
			data.Name = entry.at(L"Name").as_string();
			result->push_back(data);
		}

		return result;
	});
}

pplx::task<void> RestClient::DownloadCave(int caveId, const QString& targetFile)
{
	uri_builder builder(U("/api"));
	builder.append_path(L"Caves");
	builder.append_path(std::to_wstring(caveId));
	auto url = builder.to_string();

	return client->request(methods::GET, builder.to_string())
		.then([=](http_response response)
	{
		// Handle response headers arriving.
		if (response.status_code() != 200)
			throw NetworkException(response.status_code());

		return response.extract_string();
	}).then([=](utility::string_t str)
	{
		//decode base64
		QByteArray base64Data; base64Data.append(QString::fromStdWString(str));
		QByteArray data = QByteArray::fromBase64(base64Data);

		QFile file(targetFile);
		file.open(QIODevice::WriteOnly);
		file.write(data);
		file.close();
	});
}

pplx::task<void> RestClient::UploadSegmentation(int caveId, const std::wstring segmentationB64, const std::wstring uploaderName, int expertise, int certainty)
{
	uri_builder builder(U("/api"));
	builder.append_path(L"Caves");
	builder.append_path(std::to_wstring(caveId));
	builder.append_path(L"Segmentations");
	auto url = builder.to_string();

	web::json::value data;
	data[L"uploaderName"] = json::value::string(uploaderName);
	data[L"expertise"] = json::value::number(expertise);
	data[L"certainty"] = json::value::number(certainty);
	data[L"data"] = json::value::string(segmentationB64);

	return client->request(methods::POST, builder.to_string(), data).then(
		[=](http_response response)
	{
		if (response.status_code() != 200)
			throw NetworkException(response.status_code());
	}
	);
}