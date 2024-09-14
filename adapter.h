#pragma once

#include <string>
/* If you want to use fmt, uncomment me //如果你想使用fmt，取消注释
#include <fmt/format.h>
*/

namespace Adapter
{
    /*
    Sends a message to V5DLLAdapter. //向V5DLLAdapter发送消息。
    lpMessage: The message text, in UTF-16LE encoding. //消息文本，UTF-16LE编码。
    dwSizeInBytes: The message length in bytes. Does not include the null character. //以字节为单位的消息长度。不包含空字符。
    */
    bool SendLog(LPCWSTR lpMessage, DWORD dwSizeInBytes)
    {
        static struct {
            HWND hWnd = nullptr;
            DWORD dwMyPid = GetCurrentProcessId();
        } params;
        static const DWORD ADAPTER_MAGIC = 0x56352B2B;
        static const DWORD DATA_LOG = 1;
        if (params.hWnd == nullptr) {
            auto enumProc = [](HWND hWnd, LPARAM lParam) -> BOOL {
                auto &p = *(decltype(params) *)lParam;
                DWORD dwTargetPid = 0;
                GetWindowThreadProcessId(hWnd, &dwTargetPid);
                if (dwTargetPid == p.dwMyPid) {
                    LONG magic = GetWindowLongW(hWnd, GWLP_USERDATA);
                    if (magic == ADAPTER_MAGIC) {
                        p.hWnd = hWnd;
                        return FALSE;
                    }
                }
                return TRUE;
            };
            EnumWindows((WNDENUMPROC)enumProc, (LPARAM)&params);
        }
        if (params.hWnd != nullptr) {
            COPYDATASTRUCT cds;
            cds.dwData = DATA_LOG;
            cds.cbData = dwSizeInBytes;
            cds.lpData = (LPVOID)lpMessage;
            return SendMessageW(params.hWnd, WM_COPYDATA, NULL, (LPARAM)&cds);
        }
        return false;
    }

    //You are encouraged to create your own wrapper for this. //我们鼓励您为此创建自己的包装器

    /*
    Sends a message to V5DLLAdapter. //向V5DLLAdapter发送消息
    message: The message text.       //消息文本
    */
    bool SendLog(const std::wstring &message)
    {
        return SendLog(message.c_str(), message.size() * sizeof(wchar_t));
    }

    /* If you want to use fmt, uncomment me //如果您想使用fmt，请取消对我的注释
    template<typename ...Args>
    bool Log(fmt::WCStringRef format, Args&& ...args) {
        auto str = fmt::format(format, std::forward<Args>(args)...);
        return SendLog(str.c_str(), str.size() * sizeof(decltype(str)::value_type));
    }
    */
}  // namespace Adapter
