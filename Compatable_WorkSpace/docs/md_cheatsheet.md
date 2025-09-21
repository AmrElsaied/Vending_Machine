# Markdown Cheat Sheet

**Last Updated:** September 21, 2025  
**Purpose:** Quick reference for markdown syntax and formatting

---

## Basic Text Formatting

| Syntax | Result | Description |
|--------|--------|-------------|
| `**Bold text**` | **Bold text** | Bold formatting |
| `*Italic text*` | *Italic text* | Italic formatting |
| `***Bold and italic***` | ***Bold and italic*** | Combined bold and italic |
| `~~Strikethrough~~` | ~~Strikethrough~~ | Crossed out text |
| `` `Inline code` `` | `Inline code` | Monospace inline code |
| `<kbd>Ctrl</kbd>` | <kbd>Ctrl</kbd> | Keyboard key styling |

---

## Headers

```markdown
# H1 Header (Page title)
## H2 Header (Major section)
### H3 Header (Subsection)
#### H4 Header (Minor section)
##### H5 Header (Small section)
###### H6 Header (Smallest section)
```

### Result:
# H1 Header (Page title)
## H2 Header (Major section)
### H3 Header (Subsection)

---

## Lists

### Unordered Lists
```markdown
- Item 1
- Item 2
  - Sub-item 2.1
  - Sub-item 2.2
    - Sub-sub-item 2.2.1
- Item 3

* Alternative syntax with asterisks
+ Alternative syntax with plus signs
```

### Ordered Lists
```markdown
1. First item
2. Second item
   1. Sub-item 2.1
   2. Sub-item 2.2
3. Third item
```

### Task Lists (Checkboxes)
```markdown
- [x] Completed task
- [ ] Pending task
- [x] ~~Completed and strikethrough~~
- [ ] 🔴 **Critical priority task**
- [ ] 🟡 **High priority task**
- [ ] 🟢 **Medium priority task**
- [ ] 🔵 **Low priority task**
```

#### Result:
- [x] Completed task
- [ ] Pending task
- [x] ~~Completed and strikethrough~~
- [ ] 🔴 **Critical priority task**

---

## Links and Images

```markdown
[Simple link](https://example.com)
[Link with title](https://example.com "Hover title")
[Link to file](../Core/Inc/MDB_Handler.h)
[Link to section](#basic-text-formatting)

![Image alt text](image.png)
![Image with title](image.png "Image title")
![Image with size](<image.png> =100x50)
```

### Internal Project Links
```markdown
[`Core/Inc/MDB_Handler.h`](../Core/Inc/MDB_Handler.h)
[Configuration Guide](configuration.md)
[API Reference](api-reference.md)
```

---

## Code Blocks

### Inline Code
```markdown
Use `printf()` function in your C code
Configure the `MDB_Config_t` structure
```

### Code Blocks with Syntax Highlighting
````markdown
```c
// C code example
void MDB_BusInit(void) {
    if (mdb_config.mdb_uart == NULL) {
        // Handle error
        return;
    }
    mdbRing_init(&rxRing);
}
```

```javascript
// JavaScript example
function connectMQTT() {
    console.log("Connecting to MQTT broker");
}
```

```bash
# Shell commands
cd /path/to/project
make clean && make all
```
````

---

## Tables

### Basic Table
```markdown
| Header 1 | Header 2 | Header 3 |
|----------|----------|----------|
| Row 1    | Data     | More data|
| Row 2    | Data     | More data|
```

### Aligned Tables
```markdown
| Left-aligned | Center-aligned | Right-aligned |
|:-------------|:--------------:|--------------:|
| Left         | Center         | Right         |
| Text         | Text           | Text          |
```

#### Result:
| Function | Priority | Status |
|:---------|:--------:|-------:|
| ESP_Init | 🔴 Critical | Pending |
| MDB_BusInit | 🟡 High | Complete |

---

## Blockquotes

```markdown
> This is a blockquote
> It can span multiple lines
> 
> And include multiple paragraphs

> **Note:** Important information
> Use blockquotes for warnings or notes

>> Nested blockquote
>> For additional context
```

#### Result:
> **Note:** This is an important note about the MDB protocol implementation
> 
> Always validate UART configuration before initialization

---

## Horizontal Rules

```markdown
---
Three dashes

***
Three asterisks

___
Three underscores
```

---

## Line Breaks and Spacing

```markdown
Two spaces at end of line  
Creates a line break

Double enter

Creates a paragraph break

Use `<br>` for<br>explicit line breaks
```

---

## Emojis and Icons

### Unicode Emojis
```markdown
📡 📱 💻 🔧 ⚙️ 🔴 🟡 🟢 🔵
✅ ❌ ⚠️ 📝 📋 🔍 💡 🚀
```

### GitHub/GitLab Emoji Codes
```markdown
:smile: :heart: :thumbsup: :warning: :information_source:
:white_check_mark: :x: :heavy_check_mark: :question:
```

### Priority Icons (Project Standard)
```markdown
🔴 Critical priority
🟡 High priority  
🟢 Medium priority
🔵 Low priority
```

---

## Advanced Formatting

### Details/Collapsible Sections
```markdown
<details>
<summary>Click to expand</summary>

This content is hidden by default.

You can include:
- Lists
- Code blocks
- Any other markdown

</details>
```

#### Result:
<details>
<summary>MDB Configuration Details</summary>

```c
MDB_Config_t mdb_config = {
    .mdb_uart = &huart1,
    .debug_uart = &huart2,
    .bus_timeout = 10,
    .debug_enabled = true
};
```

</details>

### HTML in Markdown
```markdown
<kbd>Ctrl</kbd> + <kbd>C</kbd> (Keyboard shortcuts)
<mark>Highlighted text</mark>
<sub>subscript</sub> and <sup>superscript</sup>
<center>Centered text</center>
```

### Escape Characters
```markdown
\* Literal asterisk (not italic)
\_ Literal underscore
\# Literal hash (not header)
\[ Literal bracket
\\ Literal backslash
```

---

## Comments and Metadata

### Comments (Not visible in rendered markdown)
```markdown
<!-- This is a comment -->
[//]: # (This is also a comment)
[//]: # (TODO: Update this section)
```

### Document Metadata
```markdown
---
title: "Document Title"
author: "Your Name"
date: "2025-09-21"
version: "1.0"
---
```

---

## File Paths and References

### Code File References
```markdown
`/path/to/file.c`
[`filename.h`](../path/to/file.h)
See [`Core/Inc/MDB_Handler.h`](../Core/Inc/MDB_Handler.h)
Configure in [`Core/Src/MDB_PBCFG.c`](../Core/Src/MDB_PBCFG.c)
```

### Directory Structure
```markdown
```
project/
├── Core/
│   ├── Inc/
│   │   ├── MDB_Handler.h
│   │   └── ESP8266_Handler.h
│   └── Src/
│       ├── MDB_Handler.c
│       └── ESP8266_Handler.c
└── docs/
    ├── README.md
    └── TODO.md
```
```

---

## Documentation Best Practices

### Document Structure
```markdown
# Main Title

**Metadata information**

## Overview/Introduction

Brief description of the document purpose.

### Subsections

Organized content with clear hierarchy.

---

## References

Links to related documents and resources.
```

### Cross-References
```markdown
See [API Reference](api-reference.md) for detailed function documentation.
Refer to [Configuration Guide](configuration.md) for setup instructions.
Check [TODO List](TODO.md) for pending tasks.
```

### Status Indicators
```markdown
✅ **Complete** - Feature is implemented and tested
🚧 **In Progress** - Currently being developed
⏳ **Planned** - Scheduled for future development
❌ **Blocked** - Cannot proceed due to dependencies
```

---

## Quick Reference

### Most Common Syntax
| Element | Syntax |
|---------|--------|
| Bold | `**text**` |
| Italic | `*text*` |
| Code | `` `code` `` |
| Link | `[text](url)` |
| Image | `![alt](url)` |
| Header | `## Header` |
| List | `- item` |
| Task | `- [ ] task` |
| Table | `\| col1 \| col2 \|` |
| Quote | `> quote` |
| Rule | `---` |

### Project-Specific Conventions
- Use 🔴🟡🟢🔵 for priority levels
- Reference code files with backticks: `` `filename.c` ``
- Use task lists for TODO items: `- [ ] Task`
- Include file paths for code references
- Use details sections for long explanations
- Always include metadata at document top

---

**Reference:** This cheat sheet follows GitHub Flavored Markdown (GFM) syntax.