# 协作开发指南

> 本文档适用于 [rtabmap_nav2_stack](https://github.com/aqing520/rtabmap_nav2_stack.git) 项目的所有协作者。
>
> **分支规则：**
> - `main`：备份分支，**请勿直接修改**
> - `dev`：主开发分支，所有协作基于此分支进行

---

## 工作流总览

```
远端 dev 分支
      │
      │  git clone -b dev
      ▼
  本地 dev
      │
      │  git checkout -b fix/wzy-description
      ▼
  fix/xxx 分支
      │
      │  开发 → git add → git commit
      │
      │  git push -u origin fix/xxx
      ▼
  发起 Pull Request（目标：dev）
      │
      │  Review & Merge
      ▼
  合并回 dev ✓
```

---

## 第一步：克隆仓库

> 每人只需做一次。

```bash
git clone -b dev https://github.com/aqing520/rtabmap_nav2_stack.git
cd rtabmap_nav2_stack
```

克隆后会自动处于 `dev` 分支，可用以下命令确认：

```bash
git branch
# 输出: * dev
```

---

## 第二步：创建 fix 分支

**每次开发新功能或修复 bug，都要从最新的 `dev` 创建一个新的 fix 分支。**

命名规则：`fix/你的名字-简短描述`

```bash
# 先确保本地 dev 是最新的
git checkout dev
git pull origin dev

# 创建并切换到 fix 分支
git checkout -b fix/zhangsan-add-lidar-param
```

> **命名示例：**
> - `fix/zhangsan-add-lidar-param`
> - `fix/lisi-fix-launch-error`
> - `fix/wangwu-update-readme`

---

## 第三步：开发 & 提交

在 fix 分支上正常修改代码，完成后提交：

```bash
# 查看修改了哪些文件
git status

# 添加改动
git add src/your_file.cpp      

# 或者添加全部
git add .

# 提交，写清楚做了什么
git commit -m "feat: 添加激光雷达参数配置项"
```

> **commit 消息建议格式：**
> - `feat: 新增 xxx 功能`
> - `fix: 修复 xxx 问题`
> - `docs: 更新 xxx 文档`
> - `refactor: 重构 xxx 模块`

可以多次 commit，不要等所有改动都做完再一次性提交。

---

## 第四步：推送到远端

```bash
# 第一次推送，需要设置上游分支
git push -u origin fix/zhangsan-add-lidar-param

# 后续推送直接
git push
```

---

## 第五步：发起 Pull Request

1. 打开仓库页面：[https://github.com/aqing520/rtabmap_nav2_stack](https://github.com/aqing520/rtabmap_nav2_stack)
2. GitHub 会自动提示你刚推送的分支，点击 **"Compare & pull request"**
3. 确认目标分支为 **`dev`**（不要选 `master`）
4. 填写标题和描述，说明改了什么
5. 点击 **"Create pull request"**

> 提交 PR 后，如果有修改意见，在本地改完再 `git push` 即可，PR 会自动更新，无需重新提交。

---

## 第六步：等待 Review & Merge

管理员 Review 通过后会将你的分支合并到 `dev`。

合并完成后，你可以删除本地的 fix 分支：

```bash
git checkout dev
git pull origin dev
git branch -d fix/zhangsan-add-lidar-param
```

---

## 长期开发：保持与 dev 同步

如果你的 fix 分支开发时间较长，建议定期把 `dev` 的最新改动同步进来，避免冲突积累：

```bash
git fetch origin
git merge origin/dev
```

如果有冲突，解决冲突后重新 `git add` 并 `git commit` 即可。

---

## 常用命令速查

| 操作 | 命令 |
|------|------|
| 克隆 dev 分支 | `git clone -b dev https://github.com/aqing520/rtabmap_nav2_stack.git` |
| 拉取最新 dev | `git pull origin dev` |
| 创建 fix 分支 | `git checkout -b fix/你的名字-描述` |
| 查看当前状态 | `git status` |
| 添加改动 | `git add .` |
| 提交 | `git commit -m "描述"` |
| 首次推送 | `git push -u origin fix/分支名` |
| 后续推送 | `git push` |
| 查看所有分支 | `git branch -a` |
| 同步远端 dev | `git fetch origin && git merge origin/dev` |
| 删除本地分支 | `git branch -d fix/分支名` |

