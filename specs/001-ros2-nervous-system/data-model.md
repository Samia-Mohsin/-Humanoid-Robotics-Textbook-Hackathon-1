# Data Model: The Robotic Nervous System (ROS 2)

## Entity: ROS 2 Educational Content

**Description**: The core educational content entity representing the ROS 2 learning module

**Attributes**:
- **id**: Unique identifier for the module
- **title**: "The Robotic Nervous System (ROS 2)"
- **description**: Educational module covering ROS 2 fundamentals for humanoid robotics
- **target_audience**: AI students and developers entering humanoid robotics
- **version**: Current version of the content
- **status**: Draft/Published/Archived

## Entity: Chapter

**Description**: Individual chapters within the ROS 2 educational module

**Attributes**:
- **id**: Unique identifier for the chapter
- **title**: Chapter title
- **slug**: URL-friendly identifier
- **content**: Markdown content of the chapter
- **module_id**: Reference to parent module
- **order**: Sequential order in the module
- **prerequisites**: Previous chapters or knowledge required

**Relationships**:
- One module contains many chapters
- Chapters have sequential dependencies

**Validation Rules**:
- Title must be non-empty
- Order must be a positive integer
- Content must follow Docusaurus markdown standards

## Entity: ROS 2 Concept

**Description**: Individual ROS 2 concepts covered in the chapters

**Attributes**:
- **id**: Unique identifier for the concept
- **name**: Name of the ROS 2 concept (e.g., "Node", "Topic", "Service", "DDS")
- **definition**: Clear definition of the concept
- **examples**: Practical examples demonstrating the concept
- **chapter_id**: Reference to the chapter where it's taught
- **difficulty_level**: Beginner/Intermediate/Advanced

**Relationships**:
- One chapter contains many concepts
- Concepts may have prerequisite relationships

## Entity: Code Example

**Description**: Practical code examples demonstrating ROS 2 concepts

**Attributes**:
- **id**: Unique identifier for the example
- **title**: Brief description of the example
- **language**: Programming language (typically Python for rclpy)
- **code**: The actual code snippet
- **description**: Explanation of what the code does
- **concept_id**: Reference to the concept being demonstrated
- **file_path**: Where the example file is stored

**Relationships**:
- One concept may have multiple examples
- Examples are referenced within chapters

## Entity: Learning Objective

**Description**: Specific learning objectives for each chapter

**Attributes**:
- **id**: Unique identifier for the objective
- **description**: What the learner should be able to do after completing the section
- **chapter_id**: Reference to the associated chapter
- **measurable**: Whether the objective can be measured/tested
- **priority**: P1/P2/P3 indicating importance

**Validation Rules**:
- Description must be specific and measurable
- Priority must be one of P1/P2/P3

## Entity: Prerequisite

**Description**: Prerequisites required before learning specific content

**Attributes**:
- **id**: Unique identifier for the prerequisite
- **type**: Concept/Chapter/External resource
- **requirement**: What must be known/completed
- **dependent_id**: What depends on this prerequisite
- **optional**: Whether it's optional or required

## State Transitions

### Content Creation Workflow:
1. **Draft**: Content is being created and reviewed internally
2. **Review**: Content is being reviewed by subject matter experts
3. **Published**: Content is available to learners
4. **Archived**: Content is no longer current or relevant

### Validation Rules:
- Content must pass all learning objectives before publication
- All prerequisites must be satisfied before advancing to next chapter
- Code examples must be verified to work with current ROS 2 versions