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