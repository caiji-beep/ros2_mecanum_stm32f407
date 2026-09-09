#!/usr/bin/env python3

# 这个脚本的作用：
# 1. 从网络 URL 下载 STM32 app 固件，例如 GitHub raw 链接里的 slave_app.bin。
# 2. 先写入一个临时文件，下载完整并校验成功后，再替换成正式文件。
# 3. 在缓存目录里维护 latest.bin，后续 OTA 节点只需要读取 latest.bin。
#
# 对应到 C/C++ 思路：
# - pathlib.Path 类似 std::filesystem::path。
# - urllib.request.urlopen 类似发起一个 HTTP GET 请求。
# - with xxx as y 类似 C++ RAII，离开作用域自动 close 文件/网络连接。
# - raise RuntimeError 类似抛出 std::runtime_error。

import argparse
import hashlib
import shutil
import sys
import tempfile
import urllib.parse
import urllib.request
from pathlib import Path


def sha256_file(path: Path) -> str:
    """Calculate SHA256 to verify the downloaded firmware."""
    digest = hashlib.sha256()

    # "rb" 表示 read binary，按二进制方式打开文件。
    # iter(lambda: file.read(...), b"") 的意思是：
    # 每次读 1MB，直到 read() 返回空 bytes。
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)

    return digest.hexdigest()


def filename_from_url(url: str) -> str:
    """Extract the firmware filename from a URL."""
    parsed = urllib.parse.urlparse(url)
    name = Path(parsed.path).name

    # 如果 URL 末尾没有文件名，就给一个默认名字。
    return name or "firmware.bin"


def download_file(url: str, output_path: Path) -> None:
    """Download URL content to output_path."""
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # GitHub 等服务有时会检查 User-Agent。
    # 加一个明确的 User-Agent，避免被当成异常客户端。
    request = urllib.request.Request(url, headers={"User-Agent": "stm32_can_ota/0.1"})

    with urllib.request.urlopen(request, timeout=60) as response:
        with output_path.open("wb") as output:
            # shutil.copyfileobj 会循环读取 response，再写入 output。
            # 相当于 C 里 while(read(...)) write(...)。
            shutil.copyfileobj(response, output)


def main() -> int:
    # argparse 负责解析命令行参数，类似 C/C++ 里手动解析 argc/argv。
    parser = argparse.ArgumentParser(
        description="Download a firmware image to the local stm32_can_ota cache."
    )
    parser.add_argument("url", help="Firmware file URL, for example a GitHub release asset URL.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path.home() / ".stm32_can_ota" / "firmware",
        help="Local cache directory.",
    )
    parser.add_argument(
        "--filename",
        default="",
        help="Override output filename. Defaults to the filename from the URL.",
    )
    parser.add_argument(
        "--sha256",
        default="",
        help="Optional expected SHA256. If provided, the download must match it.",
    )
    args = parser.parse_args()

    # 默认缓存目录是 ~/.stm32_can_ota/firmware。
    # expanduser() 会把 ~ 展开成 /home/lsz。
    # resolve() 会转成绝对路径。
    output_dir = args.output_dir.expanduser().resolve()
    filename = args.filename or filename_from_url(args.url)
    final_path = output_dir / filename

    try:
        # 确保缓存目录存在。parents=True 相当于 mkdir -p。
        output_dir.mkdir(parents=True, exist_ok=True)

        # 先创建临时文件，例如 tmpxxxx.download。
        # 这样下载中断时不会污染正式的 slave_app.bin。
        with tempfile.NamedTemporaryFile(
            delete=False, dir=str(output_dir), suffix=".download"
        ) as temp:
            temp_path = Path(temp.name)

        try:
            # 1. 下载到临时文件。
            download_file(args.url, temp_path)

            # 2. 计算临时文件的 SHA256。
            actual_sha256 = sha256_file(temp_path)

            # 3. 如果用户传了 --sha256，就必须匹配，否则认为下载失败。
            if args.sha256 and actual_sha256.lower() != args.sha256.lower():
                raise RuntimeError(
                    f"sha256 mismatch: expected {args.sha256}, got {actual_sha256}"
                )

            # 4. 校验通过后，原子替换成正式文件。
            # 在同一个文件系统内，replace 通常是原子操作。
            temp_path.replace(final_path)

            # 5. 更新 latest.bin，让后续 OTA 永远可以用固定路径。
            # Linux 支持符号链接，所以优先创建：
            # latest.bin -> slave_app.bin
            latest_path = output_dir / "latest.bin"
            if latest_path.exists() or latest_path.is_symlink():
                latest_path.unlink()
            try:
                latest_path.symlink_to(final_path.name)
            except OSError:
                # 如果某些文件系统不支持符号链接，就退化成复制一份。
                shutil.copy2(final_path, latest_path)

            # stdout 打印正式文件路径，方便 shell 脚本接着使用。
            print(str(final_path))

            # stderr 打印辅助信息，不影响 stdout 的路径输出。
            print(f"sha256={actual_sha256}", file=sys.stderr)
            print(f"latest={latest_path}", file=sys.stderr)
            return 0
        finally:
            # 如果下载或校验中途失败，清理临时文件。
            if temp_path.exists():
                temp_path.unlink()
    except Exception as error:
        # 所有异常最后都会变成一行错误日志，并返回非 0。
        print(f"download_firmware failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    # Python 脚本入口，相当于 C/C++ 的 main()。
    sys.exit(main())
