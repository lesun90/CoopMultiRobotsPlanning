#ifndef MP_DL_HANDLER_HPP_
#define MP_DL_HANDLER_HPP_

namespace MP
{
    namespace DLHandler
    {
	void* GetSymbol(void *handle, const char * const name);
	void* GetSymbol(const char * const name);
    }
}

#endif
