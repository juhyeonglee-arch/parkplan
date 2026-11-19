"""Hugging Face Space로 올리기. 환경변수 HF_TOKEN(쓰기 권한), HF_SPACE(예: 아이디/parkplan)."""
import os
import shutil
import tempfile

from huggingface_hub import HfApi

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    token, space = os.environ["HF_TOKEN"], os.environ["HF_SPACE"]
    api = HfApi(token=token)
    api.create_repo(space, repo_type="space", space_sdk="docker", exist_ok=True)
    with tempfile.TemporaryDirectory() as d:
        shutil.copytree(os.path.join(ROOT, "parking_ai"), os.path.join(d, "parking_ai"),
                        ignore=shutil.ignore_patterns("__pycache__"))
        os.makedirs(os.path.join(d, "examples"))
        shutil.copy(os.path.join(ROOT, "examples", "예시_대지.dxf"), os.path.join(d, "examples"))
        shutil.copy(os.path.join(ROOT, "requirements.txt"), d)
        shutil.copy(os.path.join(ROOT, "deploy", "Dockerfile"), os.path.join(d, "Dockerfile"))
        shutil.copy(os.path.join(ROOT, "deploy", "space_README.md"), os.path.join(d, "README.md"))
        api.upload_folder(folder_path=d, repo_id=space, repo_type="space", delete_patterns=["*"],
                          commit_message=f"GitHub {os.environ.get('GITHUB_SHA', '')[:7]} 배포")
    print(f"https://huggingface.co/spaces/{space}")


if __name__ == "__main__":
    main()
