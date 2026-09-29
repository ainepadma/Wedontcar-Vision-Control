# 协作开发指南

本文说明如何在 `Wedontcar-Vision-Control` 项目中创建开发分支、提交代码并发起 Pull Request（PR）。为避免多人开发时互相覆盖，功能开发和修复均应通过独立分支完成，不要直接向 `main` 分支提交代码。

## 一、开始开发前

首次参与项目时，克隆仓库并进入项目目录：

```powershell
git clone https://github.com/ainepadma/Wedontcar-Vision-Control.git
cd Wedontcar-Vision-Control
```

如果本地已经有仓库，先确认没有尚未保存的修改：

```powershell
git status
```

然后切换到 `main` 并同步远端最新代码：

```powershell
git switch main
git pull --ff-only origin main
```

如果 `git status` 显示存在未提交修改，请先提交、暂存或与其他成员确认，不要直接执行可能覆盖本地修改的操作。

## 二、创建开发分支

每项功能或修复都应从最新的 `main` 创建独立分支：

```powershell
git switch -c 分支名称
```

分支统一使用以下命名格式：

```text
MMDD-GitHub用户名
```

其中，`MMDD`表示创建分支时的月日，后面填写协作者自己的GitHub用户名。当前协作者的命名示例如下：

- `0929-ainepadma`
- `0929-dadee`
- `0929-yubao-hub`

示例：

```powershell
git switch -c 0929-ainepadma
```

每位协作者应使用自己的GitHub用户名，不得使用其他成员的用户名。分支名称使用半角数字、英文字母和连字符，不使用空格或中文。

## 三、修改并检查代码

开发过程中可以随时检查修改状态：

```powershell
git status
git diff
```

提交前应确认：

1. 只包含本次任务相关的修改；
2. 没有提交密码、令牌、个人路径等敏感信息；
3. 没有提交 `Objects`、`Listings`、`.o`、`.d` 等编译产物；
4. 已完成能够执行的编译、静态检查或硬件测试；
5. `.dist` 和 `.reference` 中的原始参考资料没有被直接修改。

## 四、创建本地提交

优先精确添加本次修改的文件，不建议直接使用 `git add .`：

```powershell
git add 路径\文件名
```

查看即将提交的内容：

```powershell
git diff --cached
```

确认无误后创建提交：

```powershell
git commit -m "类型: 简要说明"
```

提交信息示例：

```text
feat: add MT9V034 image capture
fix: correct left motor PID output
docs: update hardware list
refactor: separate vision and motor control
```

一次提交尽量只处理一个明确问题。如果修改内容较多，应拆分为多个便于审查和回退的提交。

## 五、推送开发分支

第一次推送当前分支时执行：

```powershell
git push -u origin HEAD
```

设置上游分支后，后续更新只需执行：

```powershell
git push
```

不要将功能分支强制推送到 `main`，也不要使用 `git push --force` 覆盖其他成员的提交。

## 六、创建 Pull Request

推送成功后，在浏览器中打开：

<https://github.com/ainepadma/Wedontcar-Vision-Control>

GitHub 通常会显示刚推送分支的 **Compare & pull request** 按钮。点击后检查：

- **base**：`main`；
- **compare**：自己的开发分支；
- 标题能够简洁说明本次修改；
- 描述包含修改内容、修改原因和验证结果；
- 没有混入无关文件或编译产物。

推荐使用以下 PR 描述格式：

```markdown
## 修改内容

-

## 修改原因

-

## 验证情况

- [ ] 编译通过
- [ ] 静态检查通过
- [ ] 上车或硬件测试通过
- [ ] 尚未验证，原因：

## 其他说明

-
```

确认后点击 **Create pull request**。创建 PR 不代表代码已经进入 `main`；应等待审查、测试和合并。

## 七、根据审查意见更新 PR

收到修改意见后，继续在原开发分支修改并提交：

```powershell
git add 路径\文件名
git commit -m "fix: address review feedback"
git push
```

新的提交会自动出现在原 PR 中，不需要重复创建 PR。

如果开发期间 `main` 已更新，可以先同步远端，再把最新 `main` 合并到自己的分支：

```powershell
git fetch origin
git merge origin/main
```

若发生冲突，应逐项检查并解决，完成验证后再提交冲突解决结果。不要为了消除冲突而覆盖其他成员的代码。

## 八、PR 合并后

PR 合并后，本地切回并更新 `main`：

```powershell
git switch main
git pull --ff-only origin main
```

确认功能分支不再需要后，可以删除本地分支：

```powershell
git branch -d 分支名称
```

删除分支前必须确认对应 PR 已合并，且分支中没有仍需保留的未合并工作。
