---
name: java-change-impact
description: >-
  Java 專案變更影響與計畫分析技能。當使用者要求對 Java 專案進行任何修改、
  重構、新增功能、修復 Bug、刪除檔案、更新套件或調整專案結構時，必須強制觸發此 Skill。
  在取得使用者明確同意前，絕對不能直接修改、新增或刪除任何檔案與程式碼。
---

# Java Project Change Impact & Planning Skill

## Trigger Conditions

當使用者要求對 Java 專案進行以下任何操作時，**強制觸發**此 Skill：
- 修改現有程式碼或類別
- 重構（Refactor）任何模組或套件
- 新增功能、類別、介面或方法
- 修復 Bug（包含邏輯調整）
- 刪除任何檔案或類別
- 更新套件依賴（pom.xml / build.gradle）
- 調整專案目錄結構

---

## Core Principles

1. **「先分析說明，後執行變更」**：在取得使用者明確同意前，絕對不能直接修改、新增或刪除任何檔案與程式碼。
2. **「零隱藏動態」**：所有檔案讀寫、路徑異動與潛在影響，都必須完全透明地向使用者交代。

---

## Pre-Check Steps（執行計畫前預檢）

在擬定變更計畫前，先對專案進行背景檢查（**僅限唯讀操作**）：

### Step 1：Git 狀態與分支檢查
執行 `git status`，確認是否有未 commit 的變更。
- 若有未提交的變更，**必須提醒使用者先存檔或建立備份分支**，再繼續。

### Step 2：專案編譯狀態確認
確認目前專案是否能正常編譯：
- Maven 專案：`mvn compile -q`
- Gradle 專案：`gradlew check` 或 `gradlew build`
- 若有既有編譯錯誤，**必須先回報，避免將舊錯誤歸咎於新變更**。

---

## Execution Rules & Output Structure

進行任何操作前，回覆**必須嚴格包含**以下 **6 大核心區塊**：

---

### 1. 操作說明與目的（Action & Objective）

- **要做什麼事：** 具體說明即將進行的操作步驟（逐條列舉）。
- **為什麼要做：** 說明變更背後的原因、技術動機或欲解決的問題。

---

### 2. 變更影響評估（Effect Analysis）

- **程式行為改變：** 說明程式邏輯、執行行為、資料流或執行緒控制會產生什麼轉變。
- **依賴與耦合分析（Dependency Impact）：** 列出受影響的 Interface、父類別（Superclass）或被引用的類別，評估是否會牽一髮而動全身。

---

### 3. 檔案與目錄影響範圍（Directory & File Scope）

列出所有會被新增、修改或刪除的檔案與資料夾**相對路徑**，並針對每一個路徑說明理由：

| 操作 | 檔案路徑 | 原因 |
|------|----------|------|
| `[修改]` | `src/main/java/path/to/File.java` | 調整核心邏輯 |
| `[新增]` | `src/main/java/path/to/NewFile.java` | 新增獨立功能類別 |
| `[刪除]` | `src/main/java/path/to/OldFile.java` | 舊有程式碼清理 |

---

### 4. 破壞性變更警示（Breaking Changes Warning）

若涉及以下情況，需明確標示 **`[高風險破壞性變更]`** 並說明受影響的呼叫端：
- 修改公開 API（`public` 方法或類別）
- 變更方法簽名（Method Signature）
- 刪除 Class 或重命名 Package
- 修改配置檔（`application.properties` / `application.yml` / `pom.xml` / `build.gradle`）
- 修改資料庫 Schema 或 Entity 映射

若無破壞性變更，明確寫出：「✅ 本次變更無破壞性風險。」

---

### 5. Bug 與風險評估（Bug & Risk Assessment）

- **潛在 Bug 風險等級：** 高 / 中 / 低（並說明理由）
- **可能的邊界情況（Edge Cases）：** 列舉修改後可能導致崩潰或邏輯異常的情境，例如：
  - `NullPointerException`
  - 執行緒競爭（Race Condition）
  - 記憶體洩漏（Memory Leak）
  - 超時（Timeout）
  - 資料一致性問題
- **預防與測試驗證方案：** 說明如何透過單元測試、編譯檢查或模擬驗證來確保無 Bug。

---

### 6. 後續執行確認（Confirmation Request）

以明確的詢問結尾，例如：

> ✋ **請確認：** 以上計畫是否符合您的預期？是否同意按此方案執行？
> 若有任何疑慮或需要調整，請在確認前提出。

**在使用者明確回覆「同意」或「確認執行」之前，不得進行任何檔案修改。**
