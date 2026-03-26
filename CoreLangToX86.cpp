// CoreLangToX86.cpp - CoreLang to x86 Assembly Compiler with Memory Management, File System, Process Scheduler, PS/2 Mouse Support, and VESA/VBE Graphics
// Extended with EXT2/EXT3/EXT4 File System Support, PS/2 Mouse, and VESA/VBE Graphics Modes

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cctype>
#include <memory>
#include <cstring>
#include <deque>
#include <functional>

// ============================================================================
// Token Class
// ============================================================================
enum TokenType {
    TOK_IDENTIFIER,
    TOK_NUMBER,
    TOK_STRING,
    TOK_KEYWORD,
    TOK_OPERATOR,
    TOK_SYMBOL,
    TOK_COMMENT,
    TOK_EOF,
    TOK_ERROR
};

struct Token {
    TokenType type;
    std::string value;
    int line;
    int column;
    
    Token() : type(TOK_ERROR), line(0), column(0) {}
    Token(TokenType t, const std::string& v, int l, int c) 
        : type(t), value(v), line(l), column(c) {}
    
    std::string typeToString() const {
        switch(type) {
            case TOK_IDENTIFIER: return "IDENTIFIER";
            case TOK_NUMBER: return "NUMBER";
            case TOK_STRING: return "STRING";
            case TOK_KEYWORD: return "KEYWORD";
            case TOK_OPERATOR: return "OPERATOR";
            case TOK_SYMBOL: return "SYMBOL";
            case TOK_COMMENT: return "COMMENT";
            case TOK_EOF: return "EOF";
            default: return "ERROR";
        }
    }
};

// ============================================================================
// AST Node Class
// ============================================================================
struct ASTNode {
    enum NodeType {
        NODE_ROOT,
        NODE_VAR_DECL,
        NODE_FUNC_DECL,
        NODE_IF,
        NODE_ELSE,
        NODE_WHILE,
        NODE_FOR,
        NODE_RETURN,
        NODE_BINARY_OP,
        NODE_UNARY_OP,
        NODE_NUMBER,
        NODE_IDENTIFIER,
        NODE_STRING,
        NODE_CALL,
        NODE_BLOCK,
        NODE_ASSIGNMENT,
        NODE_PRINT,
        NODE_ASM,
        NODE_ARRAY_DECL,
        NODE_ARRAY_ACCESS,
        NODE_ARRAY_LENGTH,
        NODE_ARRAY_COPY,
        NODE_POINTER_DECL,
        NODE_POINTER_DEREF,
        NODE_ADDRESS_OF,
        NODE_NULL_PTR,
        NODE_PORT_IN_B,
        NODE_PORT_IN_W,
        NODE_PORT_OUT_B,
        NODE_PORT_OUT_W,
        NODE_GETCHAR,
        NODE_IS_KEY_PRESSED,
        NODE_IDT_INIT,
        NODE_IDT_SET_GATE,
        // Memory management nodes
        NODE_MALLOC,
        NODE_FREE,
        NODE_CALLOC,
        NODE_REALLOC,
        NODE_HEAP_USED,
        NODE_HEAP_FREE,
        NODE_HEAP_DUMP,
        // File system nodes (FAT12)
        NODE_FILE_OPEN,
        NODE_FILE_CLOSE,
        NODE_FILE_READ,
        NODE_FILE_WRITE,
        NODE_FILE_SEEK,
        NODE_FILE_TELL,
        NODE_FILE_DELETE,
        NODE_FILE_EXISTS,
        // EXT2/EXT3/EXT4 File System nodes
        NODE_EXT2_MOUNT,
        NODE_EXT2_OPEN,
        NODE_EXT2_READ,
        NODE_EXT2_WRITE,
        NODE_EXT2_CLOSE,
        NODE_EXT2_MKDIR,
        NODE_EXT2_STAT,
        NODE_EXT2_UNLINK,
        NODE_EXT2_RENAME,
        // Process scheduler nodes
        NODE_CREATE_PROCESS,
        NODE_CREATE_THREAD,
        NODE_YIELD,
        NODE_SLEEP,
        NODE_WAIT_PID,
        NODE_KILL_PID,
        NODE_GETPID,
        NODE_GETPPID,
        NODE_SET_PRIORITY,
        // Mouse nodes
        NODE_MOUSE_INIT,
        NODE_MOUSE_GET_X,
        NODE_MOUSE_GET_Y,
        NODE_MOUSE_GET_BUTTONS,
        NODE_MOUSE_GET_DX,
        NODE_MOUSE_GET_DY,
        NODE_MOUSE_SET_POSITION,
        NODE_MOUSE_SET_CURSOR,
        // VESA/VBE Graphics nodes
        NODE_VESA_INIT,
        NODE_VESA_SET_MODE,
        NODE_VESA_DRAW_PIXEL,
        NODE_VESA_DRAW_RECT,
        NODE_VESA_DRAW_LINE,
        NODE_VESA_DRAW_CHAR,
        NODE_VESA_DRAW_STRING,
        NODE_VESA_CLEAR,
        NODE_VESA_SWAP_BUFFERS,
        NODE_VESA_GET_WIDTH,
        NODE_VESA_GET_HEIGHT,
        NODE_VESA_GET_BPP
    };
    
    NodeType type;
    std::string value;
    std::vector<ASTNode*> children;
    std::vector<int> dimensions;
    std::vector<ASTNode*> initializers;
    bool isPointer;
    std::string pointedType;
    
    ASTNode(NodeType t, const std::string& v = "") : type(t), value(v), isPointer(false) {}
    ~ASTNode() { for(auto child : children) delete child; }
};

// ============================================================================
// Symbol Information
// ============================================================================
struct SymbolInfo {
    std::string type;
    bool isArray;
    bool isPointer;
    std::string pointedType;
    std::vector<int> dimensions;
    int size;
    int offset;
};

// ============================================================================
// Lexer Class
// ============================================================================
class Lexer {
private:
    std::string source;
    std::vector<Token> tokens;
    int position;
    int line;
    int column;
    
    bool isKeyword(const std::string& word) const {
        static const std::vector<std::string> keywords = {
            "int", "long", "string", "char", "bool", "byte", "ptr", "void",
            "func", "return", "if", "else", "while", "loop",
            "print", "printf", "error", "asm", "true", "false", "length", "copy",
            "NULL", "inb", "inw", "outb", "outw", "getchar", "is_key_pressed",
            "idt_init", "idt_set_gate",
            // Memory management keywords
            "malloc", "free", "calloc", "realloc", "heap_used", "heap_free", "heap_dump",
            // File system keywords (FAT12)
            "file_open", "file_close", "file_read", "file_write", 
            "file_seek", "file_tell", "file_delete", "file_exists",
            // EXT2/EXT3/EXT4 File System keywords
            "ext2_mount", "ext2_open", "ext2_read", "ext2_write", "ext2_close",
            "ext2_mkdir", "ext2_stat", "ext2_unlink", "ext2_rename",
            // Process scheduler keywords
            "process", "thread", "yield", "sleep", "wait", "kill",
            "create_process", "create_thread", "wait_pid", "kill_pid",
            "getpid", "getppid", "set_priority",
            // Mouse keywords
            "mouse_init", "mouse_get_x", "mouse_get_y", "mouse_get_buttons",
            "mouse_get_dx", "mouse_get_dy", "mouse_set_position", "mouse_set_cursor",
            // VESA/VBE Graphics keywords
            "vesa_init", "vesa_set_mode", "vesa_draw_pixel", "vesa_draw_rect",
            "vesa_draw_line", "vesa_draw_char", "vesa_draw_string", "vesa_clear",
            "vesa_swap_buffers", "vesa_get_width", "vesa_get_height", "vesa_get_bpp"
        };
        return std::find(keywords.begin(), keywords.end(), word) != keywords.end();
    }
    
    bool isOperator(char c) const {
        return c == '+' || c == '-' || c == '*' || c == '/' || c == '%' ||
               c == '=' || c == '!' || c == '<' || c == '>' || c == '&';
    }
    
    bool isSymbol(char c) const {
        return c == '(' || c == ')' || c == '{' || c == '}' || c == '[' || c == ']' ||
               c == ',' || c == ';' || c == ':' || c == '=';
    }
    
    void addToken(TokenType type, const std::string& value) {
        tokens.push_back(Token(type, value, line, column));
    }
    
    void error(const std::string& msg) {
        std::cerr << "Lexer Error at line " << line << ", column " << column 
                  << ": " << msg << std::endl;
        addToken(TOK_ERROR, msg);
    }
    
    std::string processEscapeSequences(const std::string& str) {
        std::string result;
        for(size_t i = 0; i < str.length(); i++) {
            if(str[i] == '\\' && i + 1 < str.length()) {
                char next = str[i + 1];
                switch(next) {
                    case 'n': result += '\n'; i++; break;
                    case 't': result += '\t'; i++; break;
                    case '\\': result += '\\'; i++; break;
                    case '"': result += '"'; i++; break;
                    default: result += str[i]; break;
                }
            } else {
                result += str[i];
            }
        }
        return result;
    }
    
public:
    Lexer() : position(0), line(1), column(1) {}
    
    std::vector<Token> tokenize(const std::string& code) {
        source = code;
        tokens.clear();
        position = 0;
        line = 1;
        column = 1;
        
        while(position < (int)source.length()) {
            char c = source[position];
            
            if(isspace(c)) {
                if(c == '\n') { line++; column = 1; }
                else { column++; }
                position++;
                continue;
            }
            
            if(c == '/' && position + 1 < (int)source.length() && source[position + 1] == '/') {
                while(position < (int)source.length() && source[position] != '\n') position++;
                line++;
                column = 1;
                continue;
            }
            
            if(c == '"') {
                std::string str;
                position++;
                column++;
                while(position < (int)source.length() && source[position] != '"') {
                    if(source[position] == '\n') {
                        error("Unterminated string literal");
                        break;
                    }
                    str += source[position];
                    position++;
                    column++;
                }
                if(position < (int)source.length() && source[position] == '"') {
                    position++;
                    column++;
                    str = processEscapeSequences(str);
                    addToken(TOK_STRING, str);
                }
                continue;
            }
            
            if(isdigit(c)) {
                std::string num;
                if(c == '0' && position + 1 < (int)source.length() && 
                   (source[position + 1] == 'x' || source[position + 1] == 'X')) {
                    num += source[position];
                    position++;
                    column++;
                    num += source[position];
                    position++;
                    column++;
                    while(position < (int)source.length() && isxdigit(source[position])) {
                        num += source[position];
                        position++;
                        column++;
                    }
                } else {
                    while(position < (int)source.length() && isdigit(source[position])) {
                        num += source[position];
                        position++;
                        column++;
                    }
                }
                addToken(TOK_NUMBER, num);
                continue;
            }
            
            if(isalpha(c) || c == '_') {
                std::string ident;
                while(position < (int)source.length() && 
                      (isalnum(source[position]) || source[position] == '_')) {
                    ident += source[position];
                    position++;
                    column++;
                }
                if(isKeyword(ident)) {
                    addToken(TOK_KEYWORD, ident);
                } else {
                    addToken(TOK_IDENTIFIER, ident);
                }
                continue;
            }
            
            if(isOperator(c)) {
                std::string op(1, c);
                position++;
                column++;
                
                if(position < (int)source.length()) {
                    char next = source[position];
                    if((c == '+' || c == '-' || c == '*' || c == '/' || c == '%' || c == '=') && next == '=') {
                        op += next;
                        position++;
                        column++;
                    } else if((c == '<' || c == '>') && next == '=') {
                        op += next;
                        position++;
                        column++;
                    } else if(c == '!' && next == '=') {
                        op += next;
                        position++;
                        column++;
                    } else if(c == '&' && next == '&') {
                        op += next;
                        position++;
                        column++;
                    }
                }
                addToken(TOK_OPERATOR, op);
                continue;
            }
            
            if(isSymbol(c)) {
                addToken(TOK_SYMBOL, std::string(1, c));
                position++;
                column++;
                continue;
            }
            
            error(std::string("Unknown character: ") + c);
            position++;
            column++;
        }
        
        addToken(TOK_EOF, "");
        return tokens;
    }
};

// ============================================================================
// Parser Class
// ============================================================================
class Parser {
private:
    std::vector<Token> tokens;
    int currentToken;
    std::vector<std::string> errors;
    
    Token& peek() { return tokens[currentToken]; }
    Token& next() { return tokens[currentToken++]; }
    bool hasNext() { return currentToken < (int)tokens.size(); }
    bool match(TokenType type, const std::string& value = "") {
        if(hasNext() && peek().type == type) {
            if(value.empty() || peek().value == value) {
                next();
                return true;
            }
        }
        return false;
    }
    
    void error(const std::string& msg) {
        std::string errMsg = "Parser Error at line " + std::to_string(peek().line) + 
                             ", column " + std::to_string(peek().column) + ": " + msg;
        errors.push_back(errMsg);
        std::cerr << errMsg << std::endl;
    }
    
    ASTNode* parseStatement() {
        if(!hasNext()) return nullptr;
        
        if(peek().type == TOK_KEYWORD) {
            std::string kw = peek().value;
            if(kw == "ptr") return parsePointerDecl();
            if(kw == "int" || kw == "long" || kw == "string" || kw == "char" ||
               kw == "bool" || kw == "byte" || kw == "void") {
                return parseVarDecl();
            }
            if(kw == "func") return parseFuncDecl();
            if(kw == "if") return parseIf();
            if(kw == "while") return parseWhile();
            if(kw == "loop") return parseFor();
            if(kw == "return") return parseReturn();
            if(kw == "print" || kw == "printf") return parsePrint();
            if(kw == "asm") return parseAsm();
            if(kw == "inb" || kw == "inw" || kw == "outb" || kw == "outw") {
                return parsePortIO();
            }
            if(kw == "getchar") return parseGetChar();
            if(kw == "is_key_pressed") return parseIsKeyPressed();
            if(kw == "idt_init") return parseIdtInit();
            if(kw == "idt_set_gate") return parseIdtSetGate();
            // Memory management functions
            if(kw == "malloc") return parseMalloc();
            if(kw == "free") return parseFree();
            if(kw == "calloc") return parseCalloc();
            if(kw == "realloc") return parseRealloc();
            if(kw == "heap_used") return parseHeapUsed();
            if(kw == "heap_free") return parseHeapFree();
            if(kw == "heap_dump") return parseHeapDump();
            // File system functions (FAT12)
            if(kw == "file_open") return parseFileOpen();
            if(kw == "file_close") return parseFileClose();
            if(kw == "file_read") return parseFileRead();
            if(kw == "file_write") return parseFileWrite();
            if(kw == "file_seek") return parseFileSeek();
            if(kw == "file_tell") return parseFileTell();
            if(kw == "file_delete") return parseFileDelete();
            if(kw == "file_exists") return parseFileExists();
            // EXT2/EXT3/EXT4 File System functions
            if(kw == "ext2_mount") return parseExt2Mount();
            if(kw == "ext2_open") return parseExt2Open();
            if(kw == "ext2_read") return parseExt2Read();
            if(kw == "ext2_write") return parseExt2Write();
            if(kw == "ext2_close") return parseExt2Close();
            if(kw == "ext2_mkdir") return parseExt2Mkdir();
            if(kw == "ext2_stat") return parseExt2Stat();
            if(kw == "ext2_unlink") return parseExt2Unlink();
            if(kw == "ext2_rename") return parseExt2Rename();
            // Process scheduler functions
            if(kw == "create_process") return parseCreateProcess();
            if(kw == "create_thread") return parseCreateThread();
            if(kw == "yield") return parseYield();
            if(kw == "sleep") return parseSleep();
            if(kw == "wait_pid") return parseWaitPid();
            if(kw == "kill_pid") return parseKillPid();
            if(kw == "getpid") return parseGetPid();
            if(kw == "getppid") return parseGetPpid();
            if(kw == "set_priority") return parseSetPriority();
            // Mouse functions
            if(kw == "mouse_init") return parseMouseInit();
            if(kw == "mouse_get_x") return parseMouseGetX();
            if(kw == "mouse_get_y") return parseMouseGetY();
            if(kw == "mouse_get_buttons") return parseMouseGetButtons();
            if(kw == "mouse_get_dx") return parseMouseGetDx();
            if(kw == "mouse_get_dy") return parseMouseGetDy();
            if(kw == "mouse_set_position") return parseMouseSetPosition();
            if(kw == "mouse_set_cursor") return parseMouseSetCursor();
            // VESA/VBE Graphics functions
            if(kw == "vesa_init") return parseVesaInit();
            if(kw == "vesa_set_mode") return parseVesaSetMode();
            if(kw == "vesa_draw_pixel") return parseVesaDrawPixel();
            if(kw == "vesa_draw_rect") return parseVesaDrawRect();
            if(kw == "vesa_draw_line") return parseVesaDrawLine();
            if(kw == "vesa_draw_char") return parseVesaDrawChar();
            if(kw == "vesa_draw_string") return parseVesaDrawString();
            if(kw == "vesa_clear") return parseVesaClear();
            if(kw == "vesa_swap_buffers") return parseVesaSwapBuffers();
            if(kw == "vesa_get_width") return parseVesaGetWidth();
            if(kw == "vesa_get_height") return parseVesaGetHeight();
            if(kw == "vesa_get_bpp") return parseVesaGetBpp();
        }
        
        if(peek().type == TOK_IDENTIFIER) {
            if(hasNext() && (tokens[currentToken + 1].value == "=" ||
               tokens[currentToken + 1].value == "[")) {
                return parseAssignment();
            }
        }
        
        if(peek().value == "*") {
            return parseAssignment();
        }
        
        error("Unexpected token: " + peek().value);
        next();
        return nullptr;
    }
    
    // VESA/VBE parsing methods
    ASTNode* parseVesaInit() {
        next(); // skip 'vesa_init'
        ASTNode* node = new ASTNode(ASTNode::NODE_VESA_INIT);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseVesaSetMode() {
        next(); // skip 'vesa_set_mode'
        ASTNode* node = new ASTNode(ASTNode::NODE_VESA_SET_MODE);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* width = parseExpression();
            if(width) node->children.push_back(width);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* height = parseExpression();
                if(height) node->children.push_back(height);
                if(peek().value == ",") {
                    next(); // skip ','
                    ASTNode* bpp = parseExpression();
                    if(bpp) node->children.push_back(bpp);
                }
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseVesaDrawPixel() {
        next(); // skip 'vesa_draw_pixel'
        ASTNode* node = new ASTNode(ASTNode::NODE_VESA_DRAW_PIXEL);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* x = parseExpression();
            if(x) node->children.push_back(x);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* y = parseExpression();
                if(y) node->children.push_back(y);
                if(peek().value == ",") {
                    next(); // skip ','
                    ASTNode* color = parseExpression();
                    if(color) node->children.push_back(color);
                }
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseVesaDrawRect() {
        next(); // skip 'vesa_draw_rect'
        ASTNode* node = new ASTNode(ASTNode::NODE_VESA_DRAW_RECT);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* x = parseExpression();
            if(x) node->children.push_back(x);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* y = parseExpression();
                if(y) node->children.push_back(y);
                if(peek().value == ",") {
                    next(); // skip ','
                    ASTNode* w = parseExpression();
                    if(w) node->children.push_back(w);
                    if(peek().value == ",") {
                        next(); // skip ','
                        ASTNode* h = parseExpression();
                        if(h) node->children.push_back(h);
                        if(peek().value == ",") {
                            next(); // skip ','
                            ASTNode* color = parseExpression();
                            if(color) node->children.push_back(color);
                        }
                    }
                }
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseVesaDrawLine() {
        next(); // skip 'vesa_draw_line'
        ASTNode* node = new ASTNode(ASTNode::NODE_VESA_DRAW_LINE);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* x1 = parseExpression();
            if(x1) node->children.push_back(x1);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* y1 = parseExpression();
                if(y1) node->children.push_back(y1);
                if(peek().value == ",") {
                    next(); // skip ','
                    ASTNode* x2 = parseExpression();
                    if(x2) node->children.push_back(x2);
                    if(peek().value == ",") {
                        next(); // skip ','
                        ASTNode* y2 = parseExpression();
                        if(y2) node->children.push_back(y2);
                        if(peek().value == ",") {
                            next(); // skip ','
                            ASTNode* color = parseExpression();
                            if(color) node->children.push_back(color);
                        }
                    }
                }
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseVesaDrawChar() {
        next(); // skip 'vesa_draw_char'
        ASTNode* node = new ASTNode(ASTNode::NODE_VESA_DRAW_CHAR);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* x = parseExpression();
            if(x) node->children.push_back(x);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* y = parseExpression();
                if(y) node->children.push_back(y);
                if(peek().value == ",") {
                    next(); // skip ','
                    ASTNode* c = parseExpression();
                    if(c) node->children.push_back(c);
                    if(peek().value == ",") {
                        next(); // skip ','
                        ASTNode* fg = parseExpression();
                        if(fg) node->children.push_back(fg);
                        if(peek().value == ",") {
                            next(); // skip ','
                            ASTNode* bg = parseExpression();
                            if(bg) node->children.push_back(bg);
                        }
                    }
                }
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseVesaDrawString() {
        next(); // skip 'vesa_draw_string'
        ASTNode* node = new ASTNode(ASTNode::NODE_VESA_DRAW_STRING);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* x = parseExpression();
            if(x) node->children.push_back(x);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* y = parseExpression();
                if(y) node->children.push_back(y);
                if(peek().value == ",") {
                    next(); // skip ','
                    ASTNode* s = parseExpression();
                    if(s) node->children.push_back(s);
                    if(peek().value == ",") {
                        next(); // skip ','
                        ASTNode* fg = parseExpression();
                        if(fg) node->children.push_back(fg);
                        if(peek().value == ",") {
                            next(); // skip ','
                            ASTNode* bg = parseExpression();
                            if(bg) node->children.push_back(bg);
                        }
                    }
                }
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseVesaClear() {
        next(); // skip 'vesa_clear'
        ASTNode* node = new ASTNode(ASTNode::NODE_VESA_CLEAR);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* color = parseExpression();
            if(color) node->children.push_back(color);
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseVesaSwapBuffers() {
        next(); // skip 'vesa_swap_buffers'
        ASTNode* node = new ASTNode(ASTNode::NODE_VESA_SWAP_BUFFERS);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseVesaGetWidth() {
        next(); // skip 'vesa_get_width'
        ASTNode* node = new ASTNode(ASTNode::NODE_VESA_GET_WIDTH);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseVesaGetHeight() {
        next(); // skip 'vesa_get_height'
        ASTNode* node = new ASTNode(ASTNode::NODE_VESA_GET_HEIGHT);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseVesaGetBpp() {
        next(); // skip 'vesa_get_bpp'
        ASTNode* node = new ASTNode(ASTNode::NODE_VESA_GET_BPP);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    // Mouse parsing methods
    ASTNode* parseMouseInit() {
        next(); // skip 'mouse_init'
        ASTNode* node = new ASTNode(ASTNode::NODE_MOUSE_INIT);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseMouseGetX() {
        next(); // skip 'mouse_get_x'
        ASTNode* node = new ASTNode(ASTNode::NODE_MOUSE_GET_X);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseMouseGetY() {
        next(); // skip 'mouse_get_y'
        ASTNode* node = new ASTNode(ASTNode::NODE_MOUSE_GET_Y);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseMouseGetButtons() {
        next(); // skip 'mouse_get_buttons'
        ASTNode* node = new ASTNode(ASTNode::NODE_MOUSE_GET_BUTTONS);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseMouseGetDx() {
        next(); // skip 'mouse_get_dx'
        ASTNode* node = new ASTNode(ASTNode::NODE_MOUSE_GET_DX);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseMouseGetDy() {
        next(); // skip 'mouse_get_dy'
        ASTNode* node = new ASTNode(ASTNode::NODE_MOUSE_GET_DY);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseMouseSetPosition() {
        next(); // skip 'mouse_set_position'
        ASTNode* node = new ASTNode(ASTNode::NODE_MOUSE_SET_POSITION);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* x = parseExpression();
            if(x) node->children.push_back(x);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* y = parseExpression();
                if(y) node->children.push_back(y);
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseMouseSetCursor() {
        next(); // skip 'mouse_set_cursor'
        ASTNode* node = new ASTNode(ASTNode::NODE_MOUSE_SET_CURSOR);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* sprite = parseExpression();
            if(sprite) node->children.push_back(sprite);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* width = parseExpression();
                if(width) node->children.push_back(width);
                if(peek().value == ",") {
                    next(); // skip ','
                    ASTNode* height = parseExpression();
                    if(height) node->children.push_back(height);
                }
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    // EXT2/EXT3/EXT4 parsing methods
    ASTNode* parseExt2Mount() {
        next(); // skip 'ext2_mount'
        ASTNode* node = new ASTNode(ASTNode::NODE_EXT2_MOUNT);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* device = parseExpression();
            if(device) node->children.push_back(device);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* mountPoint = parseExpression();
                if(mountPoint) node->children.push_back(mountPoint);
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseExt2Open() {
        next(); // skip 'ext2_open'
        ASTNode* node = new ASTNode(ASTNode::NODE_EXT2_OPEN);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* path = parseExpression();
            if(path) node->children.push_back(path);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* flags = parseExpression();
                if(flags) node->children.push_back(flags);
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseExt2Read() {
        next(); // skip 'ext2_read'
        ASTNode* node = new ASTNode(ASTNode::NODE_EXT2_READ);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* fd = parseExpression();
            if(fd) node->children.push_back(fd);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* buffer = parseExpression();
                if(buffer) node->children.push_back(buffer);
                if(peek().value == ",") {
                    next(); // skip ','
                    ASTNode* size = parseExpression();
                    if(size) node->children.push_back(size);
                }
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseExt2Write() {
        next(); // skip 'ext2_write'
        ASTNode* node = new ASTNode(ASTNode::NODE_EXT2_WRITE);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* fd = parseExpression();
            if(fd) node->children.push_back(fd);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* buffer = parseExpression();
                if(buffer) node->children.push_back(buffer);
                if(peek().value == ",") {
                    next(); // skip ','
                    ASTNode* size = parseExpression();
                    if(size) node->children.push_back(size);
                }
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseExt2Close() {
        next(); // skip 'ext2_close'
        ASTNode* node = new ASTNode(ASTNode::NODE_EXT2_CLOSE);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* fd = parseExpression();
            if(fd) node->children.push_back(fd);
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseExt2Mkdir() {
        next(); // skip 'ext2_mkdir'
        ASTNode* node = new ASTNode(ASTNode::NODE_EXT2_MKDIR);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* path = parseExpression();
            if(path) node->children.push_back(path);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* mode = parseExpression();
                if(mode) node->children.push_back(mode);
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseExt2Stat() {
        next(); // skip 'ext2_stat'
        ASTNode* node = new ASTNode(ASTNode::NODE_EXT2_STAT);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* path = parseExpression();
            if(path) node->children.push_back(path);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* statbuf = parseExpression();
                if(statbuf) node->children.push_back(statbuf);
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseExt2Unlink() {
        next(); // skip 'ext2_unlink'
        ASTNode* node = new ASTNode(ASTNode::NODE_EXT2_UNLINK);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* path = parseExpression();
            if(path) node->children.push_back(path);
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseExt2Rename() {
        next(); // skip 'ext2_rename'
        ASTNode* node = new ASTNode(ASTNode::NODE_EXT2_RENAME);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* oldPath = parseExpression();
            if(oldPath) node->children.push_back(oldPath);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* newPath = parseExpression();
                if(newPath) node->children.push_back(newPath);
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    // Process scheduler parsing methods
    ASTNode* parseCreateProcess() {
        next(); // skip 'create_process'
        ASTNode* node = new ASTNode(ASTNode::NODE_CREATE_PROCESS);
        
        if(peek().value == "(") {
            next(); // skip '('
            
            // Function pointer
            ASTNode* func = parseExpression();
            if(func) node->children.push_back(func);
            
            if(peek().value == ",") {
                next(); // skip ','
                // Process name
                ASTNode* name = parseExpression();
                if(name) node->children.push_back(name);
            }
            
            if(peek().value == ",") {
                next(); // skip ','
                // Priority
                ASTNode* priority = parseExpression();
                if(priority) node->children.push_back(priority);
            }
            
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseCreateThread() {
        next(); // skip 'create_thread'
        ASTNode* node = new ASTNode(ASTNode::NODE_CREATE_THREAD);
        
        if(peek().value == "(") {
            next(); // skip '('
            
            // Function pointer
            ASTNode* func = parseExpression();
            if(func) node->children.push_back(func);
            
            if(peek().value == ",") {
                next(); // skip ','
                // Parent PID
                ASTNode* parent = parseExpression();
                if(parent) node->children.push_back(parent);
            }
            
            if(peek().value == ",") {
                next(); // skip ','
                // Priority
                ASTNode* priority = parseExpression();
                if(priority) node->children.push_back(priority);
            }
            
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseYield() {
        next(); // skip 'yield'
        ASTNode* node = new ASTNode(ASTNode::NODE_YIELD);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseSleep() {
        next(); // skip 'sleep'
        ASTNode* node = new ASTNode(ASTNode::NODE_SLEEP);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* ms = parseExpression();
            if(ms) node->children.push_back(ms);
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseWaitPid() {
        next(); // skip 'wait_pid'
        ASTNode* node = new ASTNode(ASTNode::NODE_WAIT_PID);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* pid = parseExpression();
            if(pid) node->children.push_back(pid);
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseKillPid() {
        next(); // skip 'kill_pid'
        ASTNode* node = new ASTNode(ASTNode::NODE_KILL_PID);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* pid = parseExpression();
            if(pid) node->children.push_back(pid);
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseGetPid() {
        next(); // skip 'getpid'
        ASTNode* node = new ASTNode(ASTNode::NODE_GETPID);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseGetPpid() {
        next(); // skip 'getppid'
        ASTNode* node = new ASTNode(ASTNode::NODE_GETPPID);
        
        if(peek().value == "(") {
            next();
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseSetPriority() {
        next(); // skip 'set_priority'
        ASTNode* node = new ASTNode(ASTNode::NODE_SET_PRIORITY);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* pid = parseExpression();
            if(pid) node->children.push_back(pid);
            
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* priority = parseExpression();
                if(priority) node->children.push_back(priority);
            }
            
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    // Memory management parsing methods
    ASTNode* parseMalloc() {
        next(); // skip 'malloc'
        ASTNode* node = new ASTNode(ASTNode::NODE_MALLOC);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* size = parseExpression();
            if(size) node->children.push_back(size);
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseFree() {
        next(); // skip 'free'
        ASTNode* node = new ASTNode(ASTNode::NODE_FREE);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* ptr = parseExpression();
            if(ptr) node->children.push_back(ptr);
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseCalloc() {
        next(); // skip 'calloc'
        ASTNode* node = new ASTNode(ASTNode::NODE_CALLOC);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* count = parseExpression();
            if(count) node->children.push_back(count);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* size = parseExpression();
                if(size) node->children.push_back(size);
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseRealloc() {
        next(); // skip 'realloc'
        ASTNode* node = new ASTNode(ASTNode::NODE_REALLOC);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* ptr = parseExpression();
            if(ptr) node->children.push_back(ptr);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* newSize = parseExpression();
                if(newSize) node->children.push_back(newSize);
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseHeapUsed() {
        next(); // skip 'heap_used'
        ASTNode* node = new ASTNode(ASTNode::NODE_HEAP_USED);
        
        if(peek().value == "(") {
            next(); // skip '('
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseHeapFree() {
        next(); // skip 'heap_free'
        ASTNode* node = new ASTNode(ASTNode::NODE_HEAP_FREE);
        
        if(peek().value == "(") {
            next(); // skip '('
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseHeapDump() {
        next(); // skip 'heap_dump'
        ASTNode* node = new ASTNode(ASTNode::NODE_HEAP_DUMP);
        
        if(peek().value == "(") {
            next(); // skip '('
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    // File system parsing methods (FAT12)
    ASTNode* parseFileOpen() {
        next(); // skip 'file_open'
        ASTNode* node = new ASTNode(ASTNode::NODE_FILE_OPEN);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* name = parseExpression();
            if(name) node->children.push_back(name);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* mode = parseExpression();
                if(mode) node->children.push_back(mode);
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseFileClose() {
        next(); // skip 'file_close'
        ASTNode* node = new ASTNode(ASTNode::NODE_FILE_CLOSE);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* fd = parseExpression();
            if(fd) node->children.push_back(fd);
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseFileRead() {
        next(); // skip 'file_read'
        ASTNode* node = new ASTNode(ASTNode::NODE_FILE_READ);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* fd = parseExpression();
            if(fd) node->children.push_back(fd);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* buffer = parseExpression();
                if(buffer) node->children.push_back(buffer);
                if(peek().value == ",") {
                    next(); // skip ','
                    ASTNode* size = parseExpression();
                    if(size) node->children.push_back(size);
                }
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseFileWrite() {
        next(); // skip 'file_write'
        ASTNode* node = new ASTNode(ASTNode::NODE_FILE_WRITE);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* fd = parseExpression();
            if(fd) node->children.push_back(fd);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* buffer = parseExpression();
                if(buffer) node->children.push_back(buffer);
                if(peek().value == ",") {
                    next(); // skip ','
                    ASTNode* size = parseExpression();
                    if(size) node->children.push_back(size);
                }
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseFileSeek() {
        next(); // skip 'file_seek'
        ASTNode* node = new ASTNode(ASTNode::NODE_FILE_SEEK);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* fd = parseExpression();
            if(fd) node->children.push_back(fd);
            if(peek().value == ",") {
                next(); // skip ','
                ASTNode* pos = parseExpression();
                if(pos) node->children.push_back(pos);
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseFileTell() {
        next(); // skip 'file_tell'
        ASTNode* node = new ASTNode(ASTNode::NODE_FILE_TELL);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* fd = parseExpression();
            if(fd) node->children.push_back(fd);
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseFileDelete() {
        next(); // skip 'file_delete'
        ASTNode* node = new ASTNode(ASTNode::NODE_FILE_DELETE);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* name = parseExpression();
            if(name) node->children.push_back(name);
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseFileExists() {
        next(); // skip 'file_exists'
        ASTNode* node = new ASTNode(ASTNode::NODE_FILE_EXISTS);
        
        if(peek().value == "(") {
            next(); // skip '('
            ASTNode* name = parseExpression();
            if(name) node->children.push_back(name);
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    // Existing methods
    ASTNode* parseGetChar() {
        next();
        ASTNode* node = new ASTNode(ASTNode::NODE_GETCHAR);
        if(hasNext() && peek().value == "(") {
            next();
            if(hasNext() && peek().value == ")") next();
        }
        if(hasNext() && peek().value == ";") next();
        return node;
    }
    
    ASTNode* parseIsKeyPressed() {
        next();
        ASTNode* node = new ASTNode(ASTNode::NODE_IS_KEY_PRESSED);
        if(hasNext() && peek().value == "(") {
            next();
            if(hasNext() && peek().value == ")") next();
        }
        if(hasNext() && peek().value == ";") next();
        return node;
    }
    
    ASTNode* parseIdtInit() {
        next();
        ASTNode* node = new ASTNode(ASTNode::NODE_IDT_INIT);
        if(hasNext() && peek().value == "(") {
            next();
            if(hasNext() && peek().value == ")") next();
        }
        if(hasNext() && peek().value == ";") next();
        return node;
    }
    
    ASTNode* parseIdtSetGate() {
        next();
        ASTNode* node = new ASTNode(ASTNode::NODE_IDT_SET_GATE);
        
        if(peek().value == "(") {
            next();
            ASTNode* num = parseExpression();
            if(num) node->children.push_back(num);
            if(peek().value == ",") {
                next();
                ASTNode* handler = parseExpression();
                if(handler) node->children.push_back(handler);
                if(peek().value == ",") {
                    next();
                    ASTNode* type = parseExpression();
                    if(type) node->children.push_back(type);
                }
            }
            if(peek().value == ")") next();
        }
        
        if(hasNext() && peek().value == ";") next();
        return node;
    }
    
    ASTNode* parsePortIO() {
        std::string funcName = next().value;
        ASTNode* node = nullptr;
        
        if(funcName == "inb") node = new ASTNode(ASTNode::NODE_PORT_IN_B);
        else if(funcName == "inw") node = new ASTNode(ASTNode::NODE_PORT_IN_W);
        else if(funcName == "outb") node = new ASTNode(ASTNode::NODE_PORT_OUT_B);
        else if(funcName == "outw") node = new ASTNode(ASTNode::NODE_PORT_OUT_W);
        
        if(!node) {
            error("Unknown port I/O function");
            return nullptr;
        }
        
        if(peek().value != "(") {
            error("Expected '(' after port I/O function");
            delete node;
            return nullptr;
        }
        next();
        
        ASTNode* port = parseExpression();
        if(port) node->children.push_back(port);
        
        if(funcName == "outb" || funcName == "outw") {
            if(peek().value == ",") {
                next();
                ASTNode* value = parseExpression();
                if(value) node->children.push_back(value);
            }
        }
        
        if(peek().value == ")") next();
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parsePointerDecl() {
        next();
        std::string pointedType;
        if(peek().type == TOK_KEYWORD) {
            pointedType = next().value;
        } else {
            error("Expected type after 'ptr'");
            return nullptr;
        }
        
        if(peek().type != TOK_IDENTIFIER) {
            error("Expected pointer variable name");
            return nullptr;
        }
        std::string ptrName = next().value;
        
        ASTNode* node = new ASTNode(ASTNode::NODE_POINTER_DECL, ptrName);
        node->isPointer = true;
        node->pointedType = pointedType;
        node->children.push_back(new ASTNode(ASTNode::NODE_STRING, pointedType));
        
        if(hasNext() && peek().value == "=") {
            next();
            ASTNode* init = parseExpression();
            if(init) node->children.push_back(init);
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    std::vector<int> parseDimensions() {
        std::vector<int> dims;
        while(hasNext() && peek().value == "[") {
            next();
            if(peek().type == TOK_NUMBER) {
                dims.push_back(std::stoi(peek().value));
                next();
            } else {
                error("Expected array dimension size");
                dims.push_back(0);
            }
            if(peek().value == "]") next(); else error("Expected ']'");
        }
        return dims;
    }
    
    std::vector<ASTNode*> parseInitializerList() {
        std::vector<ASTNode*> inits;
        if(peek().value == "=") {
            next();
            if(peek().value == "{") {
                next();
                while(hasNext() && peek().value != "}") {
                    ASTNode* init = parseExpression();
                    if(init) inits.push_back(init);
                    if(peek().value == ",") next();
                }
                if(peek().value == "}") next();
            } else {
                ASTNode* init = parseExpression();
                if(init) inits.push_back(init);
            }
        }
        return inits;
    }
    
    ASTNode* parseVarDecl() {
        std::string varType = next().value;
        if(peek().type != TOK_IDENTIFIER) {
            error("Expected variable name");
            return nullptr;
        }
        std::string varName = next().value;
        
        std::vector<int> dims = parseDimensions();
        
        ASTNode* node;
        if(!dims.empty()) {
            node = new ASTNode(ASTNode::NODE_ARRAY_DECL, varName);
            node->dimensions = dims;
            node->children.push_back(new ASTNode(ASTNode::NODE_STRING, varType));
            std::vector<ASTNode*> inits = parseInitializerList();
            for(auto init : inits) node->children.push_back(init);
        } else {
            node = new ASTNode(ASTNode::NODE_VAR_DECL, varName);
            node->children.push_back(new ASTNode(ASTNode::NODE_STRING, varType));
            if(hasNext() && peek().value == "=") {
                next();
                ASTNode* init = parseExpression();
                if(init) node->children.push_back(init);
            }
        }
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseFuncDecl() {
        next();
        if(peek().type != TOK_IDENTIFIER) {
            error("Expected function name");
            return nullptr;
        }
        std::string funcName = next().value;
        
        if(peek().value != "(") {
            error("Expected (");
            return nullptr;
        }
        next();
        
        while(hasNext() && peek().value != ")") {
            if(peek().type == TOK_KEYWORD) next();
            if(peek().type == TOK_IDENTIFIER) next();
            if(peek().value == ",") next();
        }
        
        if(peek().value == ")") next();
        if(peek().value == ":") next();
        
        ASTNode* node = new ASTNode(ASTNode::NODE_FUNC_DECL, funcName);
        ASTNode* body = parseBlock();
        if(body) node->children.push_back(body);
        
        return node;
    }
    
    ASTNode* parseBlock() {
        ASTNode* block = new ASTNode(ASTNode::NODE_BLOCK);
        
        while(hasNext() && peek().value != "}") {
            ASTNode* stmt = parseStatement();
            if(stmt) block->children.push_back(stmt);
            else break;
        }
        
        if(hasNext() && peek().value == "}") next();
        
        return block;
    }
    
    ASTNode* parseIf() {
        next();
        ASTNode* ifNode = new ASTNode(ASTNode::NODE_IF);
        
        ASTNode* cond = parseExpression();
        if(cond) ifNode->children.push_back(cond);
        
        if(peek().value == ":") next();
        
        ASTNode* body = parseBlock();
        if(body) ifNode->children.push_back(body);
        
        if(hasNext() && peek().value == "else") {
            next();
            if(peek().value == ":") next();
            ASTNode* elseBody = parseBlock();
            if(elseBody) ifNode->children.push_back(elseBody);
        }
        
        return ifNode;
    }
    
    ASTNode* parseWhile() {
        next();
        ASTNode* whileNode = new ASTNode(ASTNode::NODE_WHILE);
        
        ASTNode* cond = parseExpression();
        if(cond) whileNode->children.push_back(cond);
        
        if(peek().value == ":") next();
        
        ASTNode* body = parseBlock();
        if(body) whileNode->children.push_back(body);
        
        return whileNode;
    }
    
    ASTNode* parseFor() {
        next();
        ASTNode* forNode = new ASTNode(ASTNode::NODE_FOR);
        
        if(peek().type == TOK_IDENTIFIER) {
            std::string varName = next().value;
            forNode->children.push_back(new ASTNode(ASTNode::NODE_IDENTIFIER, varName));
            if(peek().value == "=") next();
            ASTNode* init = parseExpression();
            if(init) forNode->children.push_back(init);
        }
        
        if(peek().value == ",") next();
        
        ASTNode* cond = parseExpression();
        if(cond) forNode->children.push_back(cond);
        
        if(peek().value == ",") next();
        
        ASTNode* inc = parseExpression();
        if(inc) forNode->children.push_back(inc);
        
        if(peek().value == ":") next();
        
        ASTNode* body = parseBlock();
        if(body) forNode->children.push_back(body);
        
        return forNode;
    }
    
    ASTNode* parseReturn() {
        next();
        ASTNode* retNode = new ASTNode(ASTNode::NODE_RETURN);
        
        ASTNode* expr = parseExpression();
        if(expr) retNode->children.push_back(expr);
        
        if(hasNext() && peek().value == ";") next();
        
        return retNode;
    }
    
    ASTNode* parseAssignment() {
        ASTNode* left = parsePrimary();
        
        if(peek().value == "=") {
            next();
            ASTNode* node = new ASTNode(ASTNode::NODE_ASSIGNMENT);
            node->children.push_back(left);
            ASTNode* expr = parseExpression();
            if(expr) node->children.push_back(expr);
            
            if(hasNext() && peek().value == ";") next();
            return node;
        }
        
        error("Expected '=' in assignment");
        return nullptr;
    }
    
    ASTNode* parsePrint() {
        next();
        ASTNode* node = new ASTNode(ASTNode::NODE_PRINT);
        
        ASTNode* expr = parseExpression();
        if(expr) node->children.push_back(expr);
        
        if(hasNext() && peek().value == ";") next();
        
        return node;
    }
    
    ASTNode* parseAsm() {
        next();
        ASTNode* node = new ASTNode(ASTNode::NODE_ASM);
        
        if(peek().value == "{") next();
        
        std::string asmCode;
        while(hasNext() && peek().value != "}") {
            asmCode += peek().value + " ";
            next();
        }
        
        node->children.push_back(new ASTNode(ASTNode::NODE_STRING, asmCode));
        
        if(peek().value == "}") next();
        
        return node;
    }
    
    ASTNode* parseExpression() {
        ASTNode* left = parseTerm();
        
        while(hasNext() && (peek().value == "+" || peek().value == "-" ||
              peek().value == ">" || peek().value == "<" ||
              peek().value == ">=" || peek().value == "<=" ||
              peek().value == "==" || peek().value == "!=")) {
            std::string op = next().value;
            ASTNode* right = parseTerm();
            
            ASTNode* binOp = new ASTNode(ASTNode::NODE_BINARY_OP, op);
            binOp->children.push_back(left);
            binOp->children.push_back(right);
            left = binOp;
        }
        
        return left;
    }
    
    ASTNode* parseTerm() {
        if(!hasNext()) return nullptr;
        
        ASTNode* node = parsePrimary();
        
        while(hasNext() && (peek().value == "*" || peek().value == "/" || peek().value == "%")) {
            std::string op = next().value;
            ASTNode* right = parsePrimary();
            
            ASTNode* binOp = new ASTNode(ASTNode::NODE_BINARY_OP, op);
            binOp->children.push_back(node);
            binOp->children.push_back(right);
            node = binOp;
        }
        
        return node;
    }
    
    ASTNode* parsePrimary() {
        if(!hasNext()) return nullptr;
        
        if(peek().value == "&") {
            next();
            ASTNode* operand = parsePrimary();
            ASTNode* addrNode = new ASTNode(ASTNode::NODE_ADDRESS_OF);
            addrNode->children.push_back(operand);
            return addrNode;
        }
        
        if(peek().value == "*") {
            next();
            ASTNode* operand = parsePrimary();
            ASTNode* derefNode = new ASTNode(ASTNode::NODE_POINTER_DEREF);
            derefNode->children.push_back(operand);
            return derefNode;
        }
        
        if(peek().type == TOK_NUMBER) {
            Token t = next();
            return new ASTNode(ASTNode::NODE_NUMBER, t.value);
        }
        
        if(peek().type == TOK_KEYWORD && peek().value == "NULL") {
            next();
            return new ASTNode(ASTNode::NODE_NULL_PTR, "0");
        }
        
        if(peek().type == TOK_IDENTIFIER) {
            Token t = next();
            ASTNode* node = new ASTNode(ASTNode::NODE_IDENTIFIER, t.value);
            
            if(hasNext() && peek().value == "(") {
                ASTNode* callNode = new ASTNode(ASTNode::NODE_CALL, t.value);
                next();
                
                while(hasNext() && peek().value != ")") {
                    ASTNode* arg = parseExpression();
                    if(arg) callNode->children.push_back(arg);
                    if(peek().value == ",") next();
                }
                
                if(peek().value == ")") next();
                node = callNode;
            }
            
            while(hasNext() && peek().value == "[") {
                ASTNode* access = new ASTNode(ASTNode::NODE_ARRAY_ACCESS);
                access->children.push_back(node);
                
                next();
                ASTNode* index = parseExpression();
                if(index) access->children.push_back(index);
                if(peek().value == "]") next(); else error("Expected ']'");
                
                node = access;
            }
            
            if(hasNext() && peek().value == ".") {
                next();
                if(peek().value == "length") {
                    next();
                    ASTNode* lengthNode = new ASTNode(ASTNode::NODE_ARRAY_LENGTH);
                    lengthNode->children.push_back(node);
                    node = lengthNode;
                } else if(peek().value == "copy") {
                    next();
                    if(peek().value == "(") next();
                    ASTNode* copyNode = new ASTNode(ASTNode::NODE_ARRAY_COPY);
                    copyNode->children.push_back(node);
                    ASTNode* dest = parseExpression();
                    if(dest) copyNode->children.push_back(dest);
                    if(peek().value == ")") next();
                    node = copyNode;
                }
            }
            
            return node;
        }
        
        if(peek().type == TOK_STRING) {
            Token t = next();
            return new ASTNode(ASTNode::NODE_STRING, t.value);
        }
        
        if(peek().value == "(") {
            next();
            ASTNode* expr = parseExpression();
            if(hasNext() && peek().value == ")") next();
            return expr;
        }
        
        return nullptr;
    }
    
public:
    Parser() : currentToken(0) {}
    
    ASTNode* parse(const std::vector<Token>& tokens) {
        this->tokens = tokens;
        currentToken = 0;
        errors.clear();
        
        ASTNode* root = new ASTNode(ASTNode::NODE_ROOT);
        
        while(hasNext() && peek().type != TOK_EOF) {
            ASTNode* stmt = parseStatement();
            if(stmt) root->children.push_back(stmt);
            else break;
        }
        
        return root;
    }
    
    std::vector<std::string> getErrors() const { return errors; }
};

// ============================================================================
// CodeGenerator Class - x86 Assembly Generation with Heap, File System, Scheduler, Mouse, and VESA Graphics
// Extended with EXT2/EXT3/EXT4 Support, PS/2 Mouse Support, and VESA/VBE Graphics
// ============================================================================
class CodeGenerator {
private:
    std::ostringstream output;
    std::map<std::string, SymbolInfo> symbols;
    std::vector<std::string> stringLiterals;
    int stackOffset;
    int labelCounter;
    std::string currentFunction;
    
    static const int VGA_WIDTH = 80;
    static const int VGA_HEIGHT = 25;
    
    // Heap constants
    static const int HEAP_START = 0x1000000;  // 16MB
    static const int HEAP_INIT_SIZE = 0x100000;  // 1MB initial heap
    
    // File system constants (FAT12)
    static const int MAX_OPEN_FILES = 16;
    static const int SECTOR_SIZE = 512;
    static const int FLOPPY_CYLINDERS = 80;
    static const int FLOPPY_HEADS = 2;
    static const int FLOPPY_SECTORS = 18;
    static const int FLOPPY_SIZE = 1474560;  // 1.44MB
    
    // EXT2 constants
    static const int EXT2_BLOCK_SIZE = 1024;  // Base block size
    static const int EXT2_INODE_SIZE = 128;   // Default inode size
    static const int EXT2_MAGIC = 0xEF53;
    static const int EXT2_SUPERBLOCK_OFFSET = 1024;
    static const int EXT2_MAX_OPEN = 32;
    static const int EXT2_BLOCK_CACHE_SIZE = 64;
    
    // EXT2 file types
    static const int EXT2_FT_UNKNOWN = 0;
    static const int EXT2_FT_REG_FILE = 1;
    static const int EXT2_FT_DIR = 2;
    static const int EXT2_FT_CHRDEV = 3;
    static const int EXT2_FT_BLKDEV = 4;
    static const int EXT2_FT_FIFO = 5;
    static const int EXT2_FT_SOCK = 6;
    static const int EXT2_FT_SYMLINK = 7;
    
    // EXT2 open flags
    static const int O_RDONLY = 0x0000;
    static const int O_WRONLY = 0x0001;
    static const int O_RDWR = 0x0002;
    static const int O_CREAT = 0x0100;
    static const int O_TRUNC = 0x0200;
    static const int O_APPEND = 0x0400;
    
    // Scheduler constants
    static const int MAX_PROCESSES = 64;
    static const int QUANTUM_MS = 10;  // 10ms time slice
    static const int PIT_FREQUENCY = 100;  // 100Hz
    
    // Mouse constants
    static const int MOUSE_EVENT_QUEUE_SIZE = 64;
    static const int MOUSE_IRQ = 12;
    static const int MOUSE_PORT_DATA = 0x60;
    static const int MOUSE_PORT_COMMAND = 0x64;
    static const int MOUSE_PORT_STATUS = 0x64;
    static const int MOUSE_CMD_ENABLE_AUX = 0xA8;
    static const int MOUSE_CMD_RESET = 0xFF;
    static const int MOUSE_CMD_SET_SAMPLE_RATE = 0xF3;
    static const int MOUSE_CMD_ENABLE_DATA_REPORTING = 0xF4;
    static const int MOUSE_CMD_SET_STREAM_MODE = 0xEA;
    static const int MOUSE_ACK = 0xFA;
    static const int MOUSE_RESET_ACK = 0xAA;
    
    // Mouse button masks
    static const int MOUSE_LEFT_BUTTON = 0x01;
    static const int MOUSE_RIGHT_BUTTON = 0x02;
    static const int MOUSE_MIDDLE_BUTTON = 0x04;
    
    // Mouse event types
    static const int MOUSE_EVENT_MOVE = 0x01;
    static const int MOUSE_EVENT_BUTTON_DOWN = 0x02;
    static const int MOUSE_EVENT_BUTTON_UP = 0x03;
    static const int MOUSE_EVENT_SCROLL = 0x04;
    
    // Mouse cursor defaults
    static const int MOUSE_CURSOR_WIDTH = 16;
    static const int MOUSE_CURSOR_HEIGHT = 16;
    
    // VESA/VBE Graphics constants
    static const int VESA_MODE_1024x768x32 = 0x4118;  // Common VESA mode for 1024x768x32
    static const int VESA_MODE_800x600x32 = 0x4115;   // 800x600x32
    static const int VESA_MODE_640x480x32 = 0x4112;   // 640x480x32
    static const int VESA_MODE_1024x768x16 = 0x4117;  // 1024x768x16 (565)
    static const int VESA_MODE_800x600x16 = 0x4114;   // 800x600x16
    static const int VESA_MODE_640x480x16 = 0x4111;   // 640x480x16
    static const int VESA_MODE_1024x768x8 = 0x4105;   // 1024x768x8
    static const int VESA_MODE_800x600x8 = 0x4103;    // 800x600x8
    static const int VESA_MODE_640x480x8 = 0x4101;    // 640x480x8
    
    // VBE function codes
    static const int VBE_GET_INFO = 0x4F00;
    static const int VBE_GET_MODE_INFO = 0x4F01;
    static const int VBE_SET_MODE = 0x4F02;
    
    // Font constants
    static const int FONT_WIDTH = 8;
    static const int FONT_HEIGHT = 16;
    static const int FONT_BYTES_PER_CHAR = 16;  // 8x16 = 128 bits = 16 bytes
    
    std::string newLabel() {
        return "L" + std::to_string(++labelCounter);
    }
    
    int getTypeSize(const std::string& type) {
        if(type == "int" || type == "long" || type == "bool") return 4;
        if(type == "char" || type == "byte") return 1;
        if(type == "string") return 4;
        if(type == "void") return 0;
        return 4;
    }
    
    int calculateArraySize(const std::vector<int>& dims, int elementSize) {
        int total = elementSize;
        for(int dim : dims) total *= dim;
        return total;
    }
    
    void generateMultibootHeader() {
        output << "; Multiboot header for GRUB\n";
        output << "section .multiboot\n";
        output << "align 4\n";
        output << "dd 0x1BADB002          ; Magic number\n";
        output << "dd 0x03                ; Flags (align, memory info)\n";
        output << "dd -(0x1BADB002 + 0x03) ; Checksum\n";
        output << "\n";
    }
    
    void generateEntryPoint() {
        output << "section .text\n";
        output << "global _start\n";
        output << "extern _kernel_main\n";
        output << "\n";
        output << "_start:\n";
        output << "    mov esp, _kernel_stack + 0x10000  ; Set up stack\n";
        output << "    push eax                          ; Multiboot magic\n";
        output << "    push ebx                          ; Multiboot info\n";
        output << "    call _kernel_main\n";
        output << "    cli\n";
        output << "    hlt\n";
        output << "\n";
    }
    
    void generateDataSection() {
        output << "section .data\n";
        output << "    vga_buffer equ 0xB8000\n";
        output << "    cursor_x db 0\n";
        output << "    cursor_y db 0\n";
        output << "    \n";
        output << "    ; Keyboard buffer\n";
        output << "    key_buffer db 0\n";
        output << "    key_ready db 0\n";
        output << "    \n";
        output << "    ; VESA/VBE Graphics variables\n";
        output << "    vesa_initialized db 0\n";
        output << "    vesa_framebuffer dd 0\n";
        output << "    vesa_backbuffer dd 0\n";
        output << "    vesa_width dd 0\n";
        output << "    vesa_height dd 0\n";
        output << "    vesa_bpp dd 0\n";
        output << "    vesa_pitch dd 0\n";
        output << "    vesa_mode dd 0\n";
        output << "    vesa_red_mask dd 0\n";
        output << "    vesa_green_mask dd 0\n";
        output << "    vesa_blue_mask dd 0\n";
        output << "    vesa_red_pos dd 0\n";
        output << "    vesa_green_pos dd 0\n";
        output << "    vesa_blue_pos dd 0\n";
        output << "    \n";
        output << "    ; VBE Info Block (256 bytes)\n";
        output << "    vbe_info_block:\n";
        output << "        times 256 db 0\n";
        output << "    \n";
        output << "    ; VBE Mode Info Block (256 bytes)\n";
        output << "    vbe_mode_info:\n";
        output << "        times 256 db 0\n";
        output << "    \n";
        output << "    ; Built-in 8x16 font (ASCII 32-127)\n";
        output << "    font_8x16:\n";
        generateBuiltinFont();
        output << "    \n";
        output << "    ; Mouse global state\n";
        output << "    mouse_initialized db 0\n";
        output << "    mouse_x dd 0\n";
        output << "    mouse_y dd 0\n";
        output << "    mouse_last_x dd 0\n";
        output << "    mouse_last_y dd 0\n";
        output << "    mouse_dx dd 0\n";
        output << "    mouse_dy dd 0\n";
        output << "    mouse_buttons db 0\n";
        output << "    mouse_last_buttons db 0\n";
        output << "    mouse_cycle dd 0\n";
        output << "    mouse_packet_buffer times 4 db 0\n";
        output << "    \n";
        output << "    ; Mouse event queue\n";
        output << "    mouse_event_queue times " << MOUSE_EVENT_QUEUE_SIZE * 12 << " db 0\n";
        output << "    mouse_event_head dd 0\n";
        output << "    mouse_event_tail dd 0\n";
        output << "    \n";
        output << "    ; Mouse cursor sprite (default arrow cursor)\n";
        output << "    mouse_cursor_width dd " << MOUSE_CURSOR_WIDTH << "\n";
        output << "    mouse_cursor_height dd " << MOUSE_CURSOR_HEIGHT << "\n";
        output << "    mouse_cursor_sprite times " << MOUSE_CURSOR_WIDTH * MOUSE_CURSOR_HEIGHT << " db 0\n";
        output << "    mouse_cursor_enabled db 1\n";
        output << "    \n";
        output << "    ; Mouse background buffer for cursor\n";
        output << "    mouse_bg_buffer times " << MOUSE_CURSOR_WIDTH * MOUSE_CURSOR_HEIGHT * 4 << " db 0\n";
        output << "    \n";
        output << "    ; Double-click detection\n";
        output << "    mouse_last_click_time dd 0\n";
        output << "    mouse_last_click_x dd 0\n";
        output << "    mouse_last_click_y dd 0\n";
        output << "    mouse_double_click_delay dd 250  ; 250ms\n";
        output << "    \n";
        output << "    ; IDT and keyboard interrupt\n";
        output << "    idt_ptr dw 0\n";
        output << "            dd 0\n";
        output << "    \n";
        output << "    ; PIC remap constants\n";
        output << "    PIC1_COMMAND equ 0x20\n";
        output << "    PIC1_DATA equ 0x21\n";
        output << "    PIC2_COMMAND equ 0xA0\n";
        output << "    PIC2_DATA equ 0xA1\n";
        output << "    ICW1_ICW4 equ 0x01\n";
        output << "    ICW1_INIT equ 0x10\n";
        output << "    ICW4_8086 equ 0x01\n";
        output << "    \n";
        output << "    ; PIT ports\n";
        output << "    PIT_CHANNEL0 equ 0x40\n";
        output << "    PIT_COMMAND equ 0x43\n";
        output << "    PIT_MODE equ 0x36  ; Channel 0, lobyte/hibyte, rate generator\n";
        output << "    \n";
        output << "    ; Heap management structures\n";
        output << "    heap_start dd " << std::hex << HEAP_START << std::dec << "\n";
        output << "    heap_end dd " << std::hex << (HEAP_START + HEAP_INIT_SIZE) << std::dec << "\n";
        output << "    free_list dd 0  ; Head of free list\n";
        output << "    \n";
        output << "    ; File system structures (FAT12)\n";
        output << "    open_files db " << MAX_OPEN_FILES << " dup(0)\n";
        output << "    file_positions dd " << MAX_OPEN_FILES << " dup(0)\n";
        output << "    file_sectors dd " << MAX_OPEN_FILES << " dup(0)\n";
        output << "    \n";
        output << "    ; FAT12 structures\n";
        output << "    fat12_buffer times 9216 db 0  ; 9 sectors for FAT\n";
        output << "    root_dir_buffer times 14336 db 0  ; 14 sectors for root dir\n";
        output << "    current_sector dw 0\n";
        output << "    \n";
        output << "    ; EXT2 File System structures\n";
        output << "    ext2_mounted db 0               ; Mount flag\n";
        output << "    ext2_block_size dd 1024         ; Block size in bytes\n";
        output << "    ext2_inode_size dd 128          ; Inode size\n";
        output << "    ext2_blocks_per_group dd 0      ; Blocks per group\n";
        output << "    ext2_inodes_per_group dd 0      ; Inodes per group\n";
        output << "    ext2_first_data_block dd 1      ; First data block\n";
        output << "    ext2_inode_table_offset dd 0    ; Offset to inode table\n";
        output << "    ext2_root_inode dd 2            ; Root inode number\n";
        output << "    ext2_device_start_lba dd 0      ; Device LBA start\n";
        output << "    ext2_mount_point times 256 db 0 ; Mount point path\n";
        output << "    \n";
        output << "    ; EXT2 open file table\n";
        output << "    ext2_open_files db " << EXT2_MAX_OPEN << " dup(0)\n";
        output << "    ext2_file_inodes dd " << EXT2_MAX_OPEN << " dup(0)\n";
        output << "    ext2_file_positions dd " << EXT2_MAX_OPEN << " dup(0)\n";
        output << "    ext2_file_mode dd " << EXT2_MAX_OPEN << " dup(0)\n";
        output << "    \n";
        output << "    ; EXT2 block cache (LRU)\n";
        output << "    ext2_cache_block_numbers dd " << EXT2_BLOCK_CACHE_SIZE << " dup(0)\n";
        output << "    ext2_cache_data times " << EXT2_BLOCK_CACHE_SIZE << " * 4096 db 0\n";
        output << "    ext2_cache_dirty db " << EXT2_BLOCK_CACHE_SIZE << " dup(0)\n";
        output << "    ext2_cache_lru_counter dd 0\n";
        output << "    \n";
        output << "    ; Disk I/O ports\n";
        output << "    IDE_DATA_PORT equ 0x1F0\n";
        output << "    IDE_ERROR_PORT equ 0x1F1\n";
        output << "    IDE_SECTOR_COUNT_PORT equ 0x1F2\n";
        output << "    IDE_SECTOR_NUMBER_PORT equ 0x1F3\n";
        output << "    IDE_CYLINDER_LOW_PORT equ 0x1F4\n";
        output << "    IDE_CYLINDER_HIGH_PORT equ 0x1F5\n";
        output << "    IDE_DRIVE_HEAD_PORT equ 0x1F6\n";
        output << "    IDE_STATUS_PORT equ 0x1F7\n";
        output << "    IDE_COMMAND_PORT equ 0x1F7\n";
        output << "    \n";
        output << "    ; ATA commands\n";
        output << "    ATA_CMD_READ_SECTORS equ 0x20\n";
        output << "    ATA_CMD_WRITE_SECTORS equ 0x30\n";
        output << "    \n";
        output << "    ; Scheduler structures\n";
        output << "    current_pid dd 0\n";
        output << "    quantum_counter dd 0\n";
        output << "    need_schedule db 0\n";
        output << "    \n";
        output << "    ; Process table\n";
        output << "    ; Each PCB: state(4), pid(4), priority(4), sleep_ticks(4), \n";
        output << "    ; esp(4), eip(4), eax(4), ebx(4), ecx(4), edx(4), esi(4), edi(4), ebp(4), \n";
        output << "    ; parent_pid(4), name(32), children(64*4)\n";
        output << "    PCB_SIZE equ 4*14 + 32 + 64*4  ; 56 + 32 + 256 = 344 bytes\n";
        output << "    process_table times " << MAX_PROCESSES << " * PCB_SIZE << " << " db 0\n";
        output << "    process_count dd 0\n";
        output << "    next_pid dd 1\n";
        output << "\n";
    }
    
    void generateBuiltinFont() {
        // 8x16 font for ASCII 32-127 (96 characters, 16 bytes each)
        // This is a simplified font - actual implementation would include full font data
        // For now, we'll generate a simple monospace font
        output << "    ; ASCII 32 (space)\n";
        for(int i = 0; i < 16; i++) output << "    db 0x00\n";
        output << "    ; ASCII 33-47 (punctuation)\n";
        for(int c = 33; c <= 47; c++) {
            for(int i = 0; i < 16; i++) output << "    db 0x00\n";
        }
        output << "    ; ASCII 48-57 (numbers 0-9)\n";
        for(int c = 48; c <= 57; c++) {
            for(int i = 0; i < 16; i++) output << "    db 0x00\n";
        }
        output << "    ; ASCII 58-64\n";
        for(int c = 58; c <= 64; c++) {
            for(int i = 0; i < 16; i++) output << "    db 0x00\n";
        }
        output << "    ; ASCII 65-90 (uppercase letters)\n";
        for(int c = 65; c <= 90; c++) {
            for(int i = 0; i < 16; i++) output << "    db 0x00\n";
        }
        output << "    ; ASCII 91-96\n";
        for(int c = 91; c <= 96; c++) {
            for(int i = 0; i < 16; i++) output << "    db 0x00\n";
        }
        output << "    ; ASCII 97-122 (lowercase letters)\n";
        for(int c = 97; c <= 122; c++) {
            for(int i = 0; i < 16; i++) output << "    db 0x00\n";
        }
        output << "    ; ASCII 123-127\n";
        for(int c = 123; c <= 127; c++) {
            for(int i = 0; i < 16; i++) output << "    db 0x00\n";
        }
        output << "\n";
    }
    
    void generateBssSection() {
        output << "section .bss\n";
        output << "align 16\n";
        output << "    _kernel_stack resb 0x10000\n";
        output << "    idt resb 256*8  ; 256 entries * 8 bytes each\n";
        output << "    disk_buffer resb " << SECTOR_SIZE << "\n";
        output << "    ext2_block_buffer resb 4096  ; Temporary block buffer\n";
        output << "    vesa_mode_list resb 256      ; Mode list buffer\n";
        output << "    \n";
        output << "    ; Kernel stack for interrupt handling\n";
        output << "    kernel_stack_top resb 4096\n";
        output << "    \n";
        output << "    ; Mouse cursor screen buffer\n";
        output << "    mouse_cursor_buffer resb " << MOUSE_CURSOR_WIDTH * MOUSE_CURSOR_HEIGHT * 4 << "\n";
        output << "\n";
    }
    
    void generateVesaFunctions() {
        output << "; ============================================================\n";
        output << "; VESA/VBE Graphics Functions\n";
        output << "; ============================================================\n\n";
        
        output << "; VBE function: get info\n";
        output << "_vesa_get_info:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov edi, vbe_info_block\n";
        output << "    mov word [edi], 'V'   ; VBE signature\n";
        output << "    mov word [edi+2], 'E'\n";
        output << "    mov word [edi+4], '2'\n";
        output << "    mov dword [edi+6], 0x100  ; VBE version 1.00\n";
        output << "    \n";
        output << "    ; Check if VBE is available via INT 0x10\n";
        output << "    mov ax, " << VBE_GET_INFO << "\n";
        output << "    mov di, vbe_info_block\n";
        output << "    int 0x10\n";
        output << "    \n";
        output << "    cmp ax, 0x004F\n";
        output << "    je .success\n";
        output << "    xor eax, eax\n";
        output << "    jmp .done\n";
        output << ".success:\n";
        output << "    mov eax, 1\n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VBE function: get mode info\n";
        output << "; Input: mode in [ebp+8]\n";
        output << "_vesa_get_mode_info:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov cx, [ebp+8]      ; mode number\n";
        output << "    mov di, vbe_mode_info\n";
        output << "    mov ax, " << VBE_GET_MODE_INFO << "\n";
        output << "    int 0x10\n";
        output << "    \n";
        output << "    cmp ax, 0x004F\n";
        output << "    je .success\n";
        output << "    xor eax, eax\n";
        output << "    jmp .done\n";
        output << ".success:\n";
        output << "    mov eax, 1\n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VBE function: set mode\n";
        output << "; Input: mode in [ebp+8]\n";
        output << "_vesa_set_mode:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    \n";
        output << "    mov bx, [ebp+8]      ; mode number\n";
        output << "    or bx, 0x4000        ; enable linear framebuffer\n";
        output << "    mov ax, " << VBE_SET_MODE << "\n";
        output << "    int 0x10\n";
        output << "    \n";
        output << "    cmp ax, 0x004F\n";
        output << "    je .success\n";
        output << "    xor eax, eax\n";
        output << "    jmp .done\n";
        output << ".success:\n";
        output << "    mov eax, 1\n";
        output << ".done:\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VESA init - detect and initialize VESA modes\n";
        output << "_vesa_init:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    ; Check if already initialized\n";
        output << "    cmp byte [vesa_initialized], 1\n";
        output << "    je .already_init\n";
        output << "    \n";
        output << "    ; Get VBE info\n";
        output << "    call _vesa_get_info\n";
        output << "    test eax, eax\n";
        output << "    jz .error\n";
        output << "    \n";
        output << "    ; Try preferred modes in order: 1024x768x32, 800x600x32, 640x480x32\n";
        output << "    mov esi, vesa_preferred_modes\n";
        output << ".try_modes:\n";
        output << "    mov ax, [esi]\n";
        output << "    test ax, ax\n";
        output << "    jz .error\n";
        output << "    \n";
        output << "    ; Get mode info\n";
        output << "    push eax\n";
        output << "    call _vesa_get_mode_info\n";
        output << "    add esp, 4\n";
        output << "    test eax, eax\n";
        output << "    jz .next_mode\n";
        output << "    \n";
        output << "    ; Check if mode is supported\n";
        output << "    movzx eax, word [vbe_mode_info + 0]  ; mode attributes\n";
        output << "    test eax, 0x0080      ; linear framebuffer supported?\n";
        output << "    jz .next_mode\n";
        output << "    \n";
        output << "    ; Try to set mode\n";
        output << "    push dword [esi]\n";
        output << "    call _vesa_set_mode\n";
        output << "    add esp, 4\n";
        output << "    test eax, eax\n";
        output << "    jnz .mode_set\n";
        output << "    \n";
        output << ".next_mode:\n";
        output << "    add esi, 2\n";
        output << "    jmp .try_modes\n";
        output << "    \n";
        output << ".mode_set:\n";
        output << "    ; Store mode info\n";
        output << "    mov eax, [esi]\n";
        output << "    mov [vesa_mode], eax\n";
        output << "    mov eax, [vbe_mode_info + 0x12]  ; X resolution\n";
        output << "    mov [vesa_width], eax\n";
        output << "    mov eax, [vbe_mode_info + 0x14]  ; Y resolution\n";
        output << "    mov [vesa_height], eax\n";
        output << "    mov eax, [vbe_mode_info + 0x19]  ; bits per pixel\n";
        output << "    mov [vesa_bpp], eax\n";
        output << "    mov eax, [vbe_mode_info + 0x10]  ; bytes per scan line\n";
        output << "    mov [vesa_pitch], eax\n";
        output << "    mov eax, [vbe_mode_info + 0x28]  ; physical framebuffer address\n";
        output << "    mov [vesa_framebuffer], eax\n";
        output << "    \n";
        output << "    ; Get color masks\n";
        output << "    mov eax, [vbe_mode_info + 0x1C]  ; red mask\n";
        output << "    mov [vesa_red_mask], eax\n";
        output << "    mov eax, [vbe_mode_info + 0x20]  ; green mask\n";
        output << "    mov [vesa_green_mask], eax\n";
        output << "    mov eax, [vbe_mode_info + 0x24]  ; blue mask\n";
        output << "    mov [vesa_blue_mask], eax\n";
        output << "    \n";
        output << "    ; Calculate color shift positions\n";
        output << "    mov eax, [vesa_red_mask]\n";
        output << "    bsf ecx, eax\n";
        output << "    mov [vesa_red_pos], ecx\n";
        output << "    mov eax, [vesa_green_mask]\n";
        output << "    bsf ecx, eax\n";
        output << "    mov [vesa_green_pos], ecx\n";
        output << "    mov eax, [vesa_blue_mask]\n";
        output << "    bsf ecx, eax\n";
        output << "    mov [vesa_blue_pos], ecx\n";
        output << "    \n";
        output << "    ; Allocate backbuffer\n";
        output << "    mov eax, [vesa_width]\n";
        output << "    mov ebx, [vesa_height]\n";
        output << "    mul ebx\n";
        output << "    mov ecx, [vesa_bpp]\n";
        output << "    shr ecx, 3            ; bytes per pixel\n";
        output << "    mul ecx\n";
        output << "    push eax\n";
        output << "    call _malloc\n";
        output << "    add esp, 4\n";
        output << "    test eax, eax\n";
        output << "    jz .no_backbuffer\n";
        output << "    mov [vesa_backbuffer], eax\n";
        output << "    \n";
        output << ".no_backbuffer:\n";
        output << "    ; Mark as initialized\n";
        output << "    mov byte [vesa_initialized], 1\n";
        output << "    mov eax, 1\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".already_init:\n";
        output << "    mov eax, 1\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".error:\n";
        output << "    xor eax, eax\n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VESA set mode\n";
        output << "_vesa_set_mode:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    \n";
        output << "    ; For now, just return success\n";
        output << "    ; Full implementation would find and set the requested mode\n";
        output << "    mov eax, 1\n";
        output << "    \n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VESA draw pixel\n";
        output << "; Input: x in [ebp+8], y in [ebp+12], color in [ebp+16]\n";
        output << "_vesa_draw_pixel:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    \n";
        output << "    ; Check if VESA is initialized\n";
        output << "    cmp byte [vesa_initialized], 1\n";
        output << "    jne .done\n";
        output << "    \n";
        output << "    ; Get coordinates\n";
        output << "    mov eax, [ebp + 8]   ; x\n";
        output << "    mov ebx, [ebp + 12]  ; y\n";
        output << "    mov ecx, [ebp + 16]  ; color\n";
        output << "    \n";
        output << "    ; Bounds check\n";
        output << "    cmp eax, [vesa_width]\n";
        output << "    jge .done\n";
        output << "    cmp ebx, [vesa_height]\n";
        output << "    jge .done\n";
        output << "    cmp eax, 0\n";
        output << "    jl .done\n";
        output << "    cmp ebx, 0\n";
        output << "    jl .done\n";
        output << "    \n";
        output << "    ; Calculate pixel offset\n";
        output << "    imul ebx, [vesa_pitch]  ; y * pitch\n";
        output << "    mov edx, [vesa_bpp]\n";
        output << "    shr edx, 3              ; bytes per pixel\n";
        output << "    imul eax, edx\n";
        output << "    add ebx, eax            ; offset = y*pitch + x*bpp\n";
        output << "    \n";
        output << "    ; Get framebuffer address\n";
        output << "    mov esi, [vesa_backbuffer]\n";
        output << "    test esi, esi\n";
        output << "    jz .use_frontbuffer\n";
        output << "    add esi, ebx\n";
        output << "    \n";
        output << "    ; Write pixel based on BPP\n";
        output << "    cmp dword [vesa_bpp], 32\n";
        output << "    je .write_32bit\n";
        output << "    cmp dword [vesa_bpp], 24\n";
        output << "    je .write_24bit\n";
        output << "    cmp dword [vesa_bpp], 16\n";
        output << "    je .write_16bit\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".write_32bit:\n";
        output << "    mov [esi], ecx\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".write_24bit:\n";
        output << "    mov [esi], cl\n";
        output << "    mov [esi+1], ch\n";
        output << "    shr ecx, 16\n";
        output << "    mov [esi+2], cl\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".write_16bit:\n";
        output << "    ; Convert 32-bit RGB to 16-bit RGB565\n";
        output << "    mov eax, ecx\n";
        output << "    shr eax, 3            ; Red (5 bits)\n";
        output << "    and eax, 0x1F\n";
        output << "    mov edx, ecx\n";
        output << "    shr edx, 10           ; Green (6 bits)\n";
        output << "    and edx, 0x3F\n";
        output << "    shl edx, 5\n";
        output << "    or eax, edx\n";
        output << "    mov edx, ecx\n";
        output << "    shr edx, 19           ; Blue (5 bits)\n";
        output << "    and edx, 0x1F\n";
        output << "    shl edx, 11\n";
        output << "    or eax, edx\n";
        output << "    mov [esi], ax\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".use_frontbuffer:\n";
        output << "    mov esi, [vesa_framebuffer]\n";
        output << "    add esi, ebx\n";
        output << "    cmp dword [vesa_bpp], 32\n";
        output << "    je .write_front_32bit\n";
        output << "    cmp dword [vesa_bpp], 24\n";
        output << "    je .write_front_24bit\n";
        output << "    cmp dword [vesa_bpp], 16\n";
        output << "    je .write_front_16bit\n";
        output << "    jmp .done\n";
        output << ".write_front_32bit:\n";
        output << "    mov [esi], ecx\n";
        output << "    jmp .done\n";
        output << ".write_front_24bit:\n";
        output << "    mov [esi], cl\n";
        output << "    mov [esi+1], ch\n";
        output << "    shr ecx, 16\n";
        output << "    mov [esi+2], cl\n";
        output << "    jmp .done\n";
        output << ".write_front_16bit:\n";
        output << "    ; Convert 32-bit RGB to 16-bit RGB565\n";
        output << "    mov eax, ecx\n";
        output << "    shr eax, 3\n";
        output << "    and eax, 0x1F\n";
        output << "    mov edx, ecx\n";
        output << "    shr edx, 10\n";
        output << "    and edx, 0x3F\n";
        output << "    shl edx, 5\n";
        output << "    or eax, edx\n";
        output << "    mov edx, ecx\n";
        output << "    shr edx, 19\n";
        output << "    and edx, 0x1F\n";
        output << "    shl edx, 11\n";
        output << "    or eax, edx\n";
        output << "    mov [esi], ax\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VESA draw rectangle (filled)\n";
        output << "; Input: x, y, w, h, color\n";
        output << "_vesa_draw_rect:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    ; Check if VESA is initialized\n";
        output << "    cmp byte [vesa_initialized], 1\n";
        output << "    jne .done\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]   ; x\n";
        output << "    mov ebx, [ebp + 12]  ; y\n";
        output << "    mov ecx, [ebp + 16]  ; w\n";
        output << "    mov edx, [ebp + 20]  ; h\n";
        output << "    mov esi, [ebp + 24]  ; color\n";
        output << "    \n";
        output << "    ; Bounds check\n";
        output << "    cmp eax, [vesa_width]\n";
        output << "    jge .done\n";
        output << "    cmp ebx, [vesa_height]\n";
        output << "    jge .done\n";
        output << "    \n";
        output << "    ; Clamp rectangle\n";
        output << "    mov edi, eax\n";
        output << "    add edi, ecx\n";
        output << "    cmp edi, [vesa_width]\n";
        output << "    jle .x_ok\n";
        output << "    sub edi, [vesa_width]\n";
        output << "    sub ecx, edi\n";
        output << ".x_ok:\n";
        output << "    mov edi, ebx\n";
        output << "    add edi, edx\n";
        output << "    cmp edi, [vesa_height]\n";
        output << "    jle .y_ok\n";
        output << "    sub edi, [vesa_height]\n";
        output << "    sub edx, edi\n";
        output << ".y_ok:\n";
        output << "    \n";
        output << "    ; Draw rows\n";
        output << "    mov edi, ebx          ; current y\n";
        output << ".row_loop:\n";
        output << "    cmp edi, edx\n";
        output << "    add edi, ebx\n";
        output << "    jge .done\n";
        output << "    \n";
        output << "    ; Draw horizontal line\n";
        output << "    push edi\n";
        output << "    push esi\n";
        output << "    push ecx\n";
        output << "    push edi\n";
        output << "    push eax\n";
        output << "    call _vesa_draw_hline\n";
        output << "    add esp, 20\n";
        output << "    \n";
        output << "    inc edi\n";
        output << "    jmp .row_loop\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VESA draw horizontal line\n";
        output << "_vesa_draw_hline:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]   ; x\n";
        output << "    mov ebx, [ebp + 12]  ; y\n";
        output << "    mov ecx, [ebp + 16]  ; width\n";
        output << "    mov edx, [ebp + 20]  ; color\n";
        output << "    \n";
        output << "    ; Calculate start offset\n";
        output << "    imul ebx, [vesa_pitch]\n";
        output << "    mov esi, [vesa_bpp]\n";
        output << "    shr esi, 3\n";
        output << "    imul eax, esi\n";
        output << "    add ebx, eax\n";
        output << "    \n";
        output << "    ; Get backbuffer or framebuffer\n";
        output << "    mov edi, [vesa_backbuffer]\n";
        output << "    test edi, edi\n";
        output << "    jz .use_front\n";
        output << "    add edi, ebx\n";
        output << "    jmp .draw_line\n";
        output << ".use_front:\n";
        output << "    mov edi, [vesa_framebuffer]\n";
        output << "    add edi, ebx\n";
        output << "    \n";
        output << ".draw_line:\n";
        output << "    ; Draw based on BPP\n";
        output << "    cmp dword [vesa_bpp], 32\n";
        output << "    je .draw_32bit\n";
        output << "    cmp dword [vesa_bpp], 24\n";
        output << "    je .draw_24bit\n";
        output << "    cmp dword [vesa_bpp], 16\n";
        output << "    je .draw_16bit\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".draw_32bit:\n";
        output << "    mov eax, edx\n";
        output << "    rep stosd\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".draw_24bit:\n";
        output << "    mov eax, edx\n";
        output << "    mov ebx, ecx\n";
        output << ".loop_24bit:\n";
        output << "    test ebx, ebx\n";
        output << "    jz .done\n";
        output << "    mov [edi], al\n";
        output << "    mov [edi+1], ah\n";
        output << "    shr eax, 16\n";
        output << "    mov [edi+2], al\n";
        output << "    shl eax, 16\n";
        output << "    add edi, 3\n";
        output << "    dec ebx\n";
        output << "    jmp .loop_24bit\n";
        output << "    \n";
        output << ".draw_16bit:\n";
        output << "    ; Convert color to RGB565\n";
        output << "    mov eax, edx\n";
        output << "    shr eax, 3\n";
        output << "    and eax, 0x1F\n";
        output << "    mov ebx, edx\n";
        output << "    shr ebx, 10\n";
        output << "    and ebx, 0x3F\n";
        output << "    shl ebx, 5\n";
        output << "    or eax, ebx\n";
        output << "    mov ebx, edx\n";
        output << "    shr ebx, 19\n";
        output << "    and ebx, 0x1F\n";
        output << "    shl ebx, 11\n";
        output << "    or eax, ebx\n";
        output << "    rep stosw\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VESA draw line (Bresenham)\n";
        output << "; Input: x1, y1, x2, y2, color\n";
        output << "_vesa_draw_line:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]   ; x1\n";
        output << "    mov ebx, [ebp + 12]  ; y1\n";
        output << "    mov ecx, [ebp + 16]  ; x2\n";
        output << "    mov edx, [ebp + 20]  ; y2\n";
        output << "    mov esi, [ebp + 24]  ; color\n";
        output << "    \n";
        output << "    ; Calculate deltas\n";
        output << "    sub ecx, eax         ; dx = x2 - x1\n";
        output << "    sub edx, ebx         ; dy = y2 - y1\n";
        output << "    \n";
        output << "    ; Determine step directions\n";
        output << "    cmp ecx, 0\n";
        output << "    jge .x_positive\n";
        output << "    neg ecx\n";
        output << "    mov edi, -1\n";
        output << "    jmp .x_step_set\n";
        output << ".x_positive:\n";
        output << "    mov edi, 1\n";
        output << ".x_step_set:\n";
        output << "    \n";
        output << "    cmp edx, 0\n";
        output << "    jge .y_positive\n";
        output << "    neg edx\n";
        output << "    mov ebp, -1\n";
        output << "    jmp .y_step_set\n";
        output << ".y_positive:\n";
        output << "    mov ebp, 1\n";
        output << ".y_step_set:\n";
        output << "    \n";
        output << "    ; Bresenham algorithm\n";
        output << "    cmp ecx, edx\n";
        output << "    jge .x_major\n";
        output << "    ; y-major line\n";
        output << "    mov [line_x], eax\n";
        output << "    mov [line_y], ebx\n";
        output << "    mov eax, ecx\n";
        output << "    shl eax, 1\n";
        output << "    mov [line_err], eax\n";
        output << "    mov eax, edx\n";
        output << "    shl eax, 1\n";
        output << "    sub [line_err], eax\n";
        output << "    mov [line_dy], edx\n";
        output << "    \n";
        output << "    mov ecx, [line_dy]\n";
        output << "    inc ecx\n";
        output << ".y_major_loop:\n";
        output << "    test ecx, ecx\n";
        output << "    jz .done\n";
        output << "    push ecx\n";
        output << "    push esi\n";
        output << "    push dword [line_y]\n";
        output << "    push dword [line_x]\n";
        output << "    call _vesa_draw_pixel\n";
        output << "    add esp, 12\n";
        output << "    pop ecx\n";
        output << "    \n";
        output << "    cmp dword [line_err], 0\n";
        output << "    jge .y_major_adjust\n";
        output << "    add [line_err], dword [line_dy]\n";
        output << "    shl [line_dy], 1\n";
        output << "    jmp .y_major_next\n";
        output << ".y_major_adjust:\n";
        output << "    add [line_x], edi\n";
        output << "    sub [line_err], dword [line_dy]\n";
        output << ".y_major_next:\n";
        output << "    add [line_y], ebp\n";
        output << "    dec ecx\n";
        output << "    jmp .y_major_loop\n";
        output << "    \n";
        output << ".x_major:\n";
        output << "    ; x-major line\n";
        output << "    mov [line_x], eax\n";
        output << "    mov [line_y], ebx\n";
        output << "    mov eax, edx\n";
        output << "    shl eax, 1\n";
        output << "    mov [line_err], eax\n";
        output << "    mov eax, ecx\n";
        output << "    shl eax, 1\n";
        output << "    sub [line_err], eax\n";
        output << "    mov [line_dx], ecx\n";
        output << "    \n";
        output << "    mov ecx, [line_dx]\n";
        output << "    inc ecx\n";
        output << ".x_major_loop:\n";
        output << "    test ecx, ecx\n";
        output << "    jz .done\n";
        output << "    push ecx\n";
        output << "    push esi\n";
        output << "    push dword [line_y]\n";
        output << "    push dword [line_x]\n";
        output << "    call _vesa_draw_pixel\n";
        output << "    add esp, 12\n";
        output << "    pop ecx\n";
        output << "    \n";
        output << "    cmp dword [line_err], 0\n";
        output << "    jge .x_major_adjust\n";
        output << "    add [line_err], dword [line_dx]\n";
        output << "    shl [line_dx], 1\n";
        output << "    jmp .x_major_next\n";
        output << ".x_major_adjust:\n";
        output << "    add [line_y], ebp\n";
        output << "    sub [line_err], dword [line_dx]\n";
        output << ".x_major_next:\n";
        output << "    add [line_x], edi\n";
        output << "    dec ecx\n";
        output << "    jmp .x_major_loop\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VESA draw character\n";
        output << "; Input: x, y, char, fg, bg\n";
        output << "_vesa_draw_char:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    ; Check if VESA is initialized\n";
        output << "    cmp byte [vesa_initialized], 1\n";
        output << "    jne .done\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]   ; x\n";
        output << "    mov ebx, [ebp + 12]  ; y\n";
        output << "    movzx ecx, byte [ebp + 16]  ; character\n";
        output << "    mov edx, [ebp + 20]  ; foreground color\n";
        output << "    mov esi, [ebp + 24]  ; background color\n";
        output << "    \n";
        output << "    ; Get font data\n";
        output << "    sub ecx, 32          ; ASCII offset\n";
        output << "    cmp ecx, 0\n";
        output << "    jl .done\n";
        output << "    cmp ecx, 96\n";
        output << "    jge .done\n";
        output << "    \n";
        output << "    imul ecx, " << FONT_BYTES_PER_CHAR << "\n";
        output << "    add ecx, font_8x16\n";
        output << "    \n";
        output << "    ; Draw character rows\n";
        output << "    mov edi, 0           ; row counter\n";
        output << ".row_loop:\n";
        output << "    cmp edi, " << FONT_HEIGHT << "\n";
        output << "    jge .done\n";
        output << "    \n";
        output << "    movzx ebx, byte [ecx + edi]  ; font row data\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    ; Draw row pixels\n";
        output << "    mov edi, 0           ; column counter\n";
        output << ".col_loop:\n";
        output << "    cmp edi, " << FONT_WIDTH << "\n";
        output << "    jge .next_row\n";
        output << "    \n";
        output << "    test ebx, 1\n";
        output << "    jz .bg_pixel\n";
        output << "    ; Foreground pixel\n";
        output << "    push edx\n";
        output << "    jmp .draw_pixel\n";
        output << ".bg_pixel:\n";
        output << "    push esi\n";
        output << ".draw_pixel:\n";
        output << "    push dword [ebp + 12]\n";
        output << "    add dword [esp], edi\n";
        output << "    push dword [ebp + 8]\n";
        output << "    add dword [esp], dword [esp + 4]\n";
        output << "    call _vesa_draw_pixel\n";
        output << "    add esp, 12\n";
        output << "    \n";
        output << "    shr ebx, 1\n";
        output << "    inc edi\n";
        output << "    jmp .col_loop\n";
        output << "    \n";
        output << ".next_row:\n";
        output << "    pop edi\n";
        output << "    inc edi\n";
        output << "    add dword [ebp + 12], 1  ; y++\n";
        output << "    jmp .row_loop\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VESA draw string\n";
        output << "; Input: x, y, string pointer, fg, bg\n";
        output << "_vesa_draw_string:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]   ; x\n";
        output << "    mov ebx, [ebp + 12]  ; y\n";
        output << "    mov esi, [ebp + 16]  ; string pointer\n";
        output << "    mov edx, [ebp + 20]  ; fg\n";
        output << "    mov ecx, [ebp + 24]  ; bg\n";
        output << "    \n";
        output << "    mov edi, eax         ; current x\n";
        output << "    \n";
        output << ".char_loop:\n";
        output << "    mov al, [esi]\n";
        output << "    test al, al\n";
        output << "    jz .done\n";
        output << "    \n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push edi\n";
        output << "    call _vesa_draw_char\n";
        output << "    add esp, 20\n";
        output << "    \n";
        output << "    add edi, " << FONT_WIDTH << "\n";
        output << "    inc esi\n";
        output << "    jmp .char_loop\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VESA clear screen\n";
        output << "; Input: color\n";
        output << "_vesa_clear:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    ; Check if VESA is initialized\n";
        output << "    cmp byte [vesa_initialized], 1\n";
        output << "    jne .done\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]   ; color\n";
        output << "    mov ecx, [vesa_width]\n";
        output << "    mov edx, [vesa_height]\n";
        output << "    mul ecx\n";
        output << "    mov ecx, eax          ; total pixels\n";
        output << "    \n";
        output << "    ; Get backbuffer or framebuffer\n";
        output << "    mov edi, [vesa_backbuffer]\n";
        output << "    test edi, edi\n";
        output << "    jz .use_front\n";
        output << "    \n";
        output << "    ; Fill backbuffer based on BPP\n";
        output << "    cmp dword [vesa_bpp], 32\n";
        output << "    je .fill_32bit\n";
        output << "    cmp dword [vesa_bpp], 24\n";
        output << "    je .fill_24bit\n";
        output << "    cmp dword [vesa_bpp], 16\n";
        output << "    je .fill_16bit\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".fill_32bit:\n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    rep stosd\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".fill_24bit:\n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    mov ebx, ecx\n";
        output << ".loop_24bit:\n";
        output << "    test ebx, ebx\n";
        output << "    jz .done\n";
        output << "    mov [edi], al\n";
        output << "    mov [edi+1], ah\n";
        output << "    shr eax, 16\n";
        output << "    mov [edi+2], al\n";
        output << "    shl eax, 16\n";
        output << "    add edi, 3\n";
        output << "    dec ebx\n";
        output << "    jmp .loop_24bit\n";
        output << "    \n";
        output << ".fill_16bit:\n";
        output << "    ; Convert color to RGB565\n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    shr eax, 3\n";
        output << "    and eax, 0x1F\n";
        output << "    mov ebx, [ebp + 8]\n";
        output << "    shr ebx, 10\n";
        output << "    and ebx, 0x3F\n";
        output << "    shl ebx, 5\n";
        output << "    or eax, ebx\n";
        output << "    mov ebx, [ebp + 8]\n";
        output << "    shr ebx, 19\n";
        output << "    and ebx, 0x1F\n";
        output << "    shl ebx, 11\n";
        output << "    or eax, ebx\n";
        output << "    rep stosw\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".use_front:\n";
        output << "    mov edi, [vesa_framebuffer]\n";
        output << "    cmp dword [vesa_bpp], 32\n";
        output << "    je .fill_32bit\n";
        output << "    cmp dword [vesa_bpp], 24\n";
        output << "    je .fill_24bit\n";
        output << "    cmp dword [vesa_bpp], 16\n";
        output << "    je .fill_16bit\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VESA swap buffers\n";
        output << "_vesa_swap_buffers:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    ; Check if backbuffer exists\n";
        output << "    mov esi, [vesa_backbuffer]\n";
        output << "    test esi, esi\n";
        output << "    jz .done\n";
        output << "    \n";
        output << "    ; Calculate total bytes to copy\n";
        output << "    mov eax, [vesa_width]\n";
        output << "    mov ebx, [vesa_height]\n";
        output << "    mul ebx\n";
        output << "    mov ecx, [vesa_bpp]\n";
        output << "    shr ecx, 3\n";
        output << "    mul ecx\n";
        output << "    mov ecx, eax\n";
        output << "    \n";
        output << "    ; Copy backbuffer to framebuffer\n";
        output << "    mov edi, [vesa_framebuffer]\n";
        output << "    cld\n";
        output << "    rep movsb\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VESA get width\n";
        output << "_vesa_get_width:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    mov eax, [vesa_width]\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VESA get height\n";
        output << "_vesa_get_height:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    mov eax, [vesa_height]\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; VESA get bits per pixel\n";
        output << "_vesa_get_bpp:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    mov eax, [vesa_bpp]\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "section .data\n";
        output << "vesa_preferred_modes:\n";
        output << "    dw " << VESA_MODE_1024x768x32 << "\n";
        output << "    dw " << VESA_MODE_800x600x32 << "\n";
        output << "    dw " << VESA_MODE_640x480x32 << "\n";
        output << "    dw " << VESA_MODE_1024x768x16 << "\n";
        output << "    dw " << VESA_MODE_800x600x16 << "\n";
        output << "    dw " << VESA_MODE_640x480x16 << "\n";
        output << "    dw 0\n";
        output << "\n";
        
        output << "section .bss\n";
        output << "line_x resd 1\n";
        output << "line_y resd 1\n";
        output << "line_err resd 1\n";
        output << "line_dx resd 1\n";
        output << "line_dy resd 1\n";
        output << "\n";
    }
    
    void generateMouseFunctions() {
        output << "; ============================================================\n";
        output << "; PS/2 Mouse Functions\n";
        output << "; ============================================================\n\n";
        
        output << "; Wait for mouse acknowledge\n";
        output << "_mouse_wait_ack:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ecx\n";
        output << "    \n";
        output << "    mov ecx, 100000\n";
        output << ".wait_loop:\n";
        output << "    in al, " << MOUSE_PORT_STATUS << "\n";
        output << "    test al, 0x01        ; Data ready?\n";
        output << "    jnz .check_ack\n";
        output << "    loop .wait_loop\n";
        output << "    jmp .timeout\n";
        output << ".check_ack:\n";
        output << "    in al, " << MOUSE_PORT_DATA << "\n";
        output << "    cmp al, " << MOUSE_ACK << "\n";
        output << "    je .success\n";
        output << "    cmp al, " << MOUSE_RESET_ACK << "\n";
        output << "    je .success\n";
        output << ".timeout:\n";
        output << "    xor eax, eax\n";
        output << "    jmp .done\n";
        output << ".success:\n";
        output << "    mov eax, 1\n";
        output << ".done:\n";
        output << "    pop ecx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Send command to mouse\n";
        output << "; Input: command in al\n";
        output << "_mouse_send_command:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ecx\n";
        output << "    \n";
        output << "    ; Wait for input buffer empty\n";
        output << "    mov ecx, 100000\n";
        output << ".wait_input:\n";
        output << "    in al, " << MOUSE_PORT_STATUS << "\n";
        output << "    test al, 0x02\n";
        output << "    jz .input_ready\n";
        output << "    loop .wait_input\n";
        output << "    jmp .error\n";
        output << ".input_ready:\n";
        output << "    ; Send command to mouse\n";
        output << "    mov al, 0xD4        ; Write to mouse\n";
        output << "    out " << MOUSE_PORT_COMMAND << ", al\n";
        output << "    \n";
        output << "    ; Wait for input buffer empty\n";
        output << "    mov ecx, 100000\n";
        output << ".wait_input2:\n";
        output << "    in al, " << MOUSE_PORT_STATUS << "\n";
        output << "    test al, 0x02\n";
        output << "    jz .input_ready2\n";
        output << "    loop .wait_input2\n";
        output << "    jmp .error\n";
        output << ".input_ready2:\n";
        output << "    ; Send actual command\n";
        output << "    mov al, [ebp + 8]\n";
        output << "    out " << MOUSE_PORT_DATA << ", al\n";
        output << "    \n";
        output << "    ; Wait for ACK\n";
        output << "    call _mouse_wait_ack\n";
        output << "    test eax, eax\n";
        output << "    jnz .success\n";
        output << ".error:\n";
        output << "    xor eax, eax\n";
        output << "    jmp .done\n";
        output << ".success:\n";
        output << "    mov eax, 1\n";
        output << ".done:\n";
        output << "    pop ecx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Initialize PS/2 mouse\n";
        output << "_mouse_init:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    \n";
        output << "    ; Check if already initialized\n";
        output << "    cmp byte [mouse_initialized], 1\n";
        output << "    je .done\n";
        output << "    \n";
        output << "    ; Enable auxiliary device (PS/2 mouse)\n";
        output << "    mov al, " << MOUSE_CMD_ENABLE_AUX << "\n";
        output << "    out " << MOUSE_PORT_COMMAND << ", al\n";
        output << "    \n";
        output << "    ; Wait for input buffer empty\n";
        output << "    mov ecx, 100000\n";
        output << ".wait_aux:\n";
        output << "    in al, " << MOUSE_PORT_STATUS << "\n";
        output << "    test al, 0x02\n";
        output << "    jz .aux_ready\n";
        output << "    loop .wait_aux\n";
        output << "    jmp .error\n";
        output << ".aux_ready:\n";
        output << "    \n";
        output << "    ; Send reset command\n";
        output << "    push dword " << MOUSE_CMD_RESET << "\n";
        output << "    call _mouse_send_command\n";
        output << "    add esp, 4\n";
        output << "    test eax, eax\n";
        output << "    jz .error\n";
        output << "    \n";
        output << "    ; Set sample rate to 100Hz\n";
        output << "    push dword " << MOUSE_CMD_SET_SAMPLE_RATE << "\n";
        output << "    call _mouse_send_command\n";
        output << "    add esp, 4\n";
        output << "    test eax, eax\n";
        output << "    jz .error\n";
        output << "    \n";
        output << "    push dword 100\n";
        output << "    call _mouse_send_command\n";
        output << "    add esp, 4\n";
        output << "    test eax, eax\n";
        output << "    jz .error\n";
        output << "    \n";
        output << "    ; Set stream mode\n";
        output << "    push dword " << MOUSE_CMD_SET_STREAM_MODE << "\n";
        output << "    call _mouse_send_command\n";
        output << "    add esp, 4\n";
        output << "    test eax, eax\n";
        output << "    jz .error\n";
        output << "    \n";
        output << "    ; Enable data reporting\n";
        output << "    push dword " << MOUSE_CMD_ENABLE_DATA_REPORTING << "\n";
        output << "    call _mouse_send_command\n";
        output << "    add esp, 4\n";
        output << "    test eax, eax\n";
        output << "    jz .error\n";
        output << "    \n";
        output << "    ; Initialize mouse state\n";
        output << "    mov dword [mouse_x], " << (VGA_WIDTH * 8) << "\n";
        output << "    mov dword [mouse_y], " << (VGA_HEIGHT * 8) << "\n";
        output << "    mov dword [mouse_last_x], " << (VGA_WIDTH * 8) << "\n";
        output << "    mov dword [mouse_last_y], " << (VGA_HEIGHT * 8) << "\n";
        output << "    mov dword [mouse_dx], 0\n";
        output << "    mov dword [mouse_dy], 0\n";
        output << "    mov byte [mouse_buttons], 0\n";
        output << "    mov byte [mouse_last_buttons], 0\n";
        output << "    mov dword [mouse_cycle], 0\n";
        output << "    \n";
        output << "    ; Initialize default cursor sprite (arrow)\n";
        output << "    call _mouse_init_default_cursor\n";
        output << "    \n";
        output << "    ; Mark as initialized\n";
        output << "    mov byte [mouse_initialized], 1\n";
        output << "    mov eax, 1\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".error:\n";
        output << "    xor eax, eax\n";
        output << ".done:\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Initialize default arrow cursor sprite\n";
        output << "_mouse_init_default_cursor:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    ; Default arrow cursor pattern (16x16)\n";
        output << "    ; 1 = white, 2 = black, 0 = transparent\n";
        output << "    mov esi, default_cursor_pattern\n";
        output << "    mov edi, mouse_cursor_sprite\n";
        output << "    mov ecx, " << MOUSE_CURSOR_WIDTH * MOUSE_CURSOR_HEIGHT << "\n";
        output << "    rep movsb\n";
        output << "    \n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Set custom cursor sprite\n";
        output << "; Input: sprite_data in [ebp+8], width in [ebp+12], height in [ebp+16]\n";
        output << "_mouse_set_cursor:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov esi, [ebp + 8]     ; sprite data\n";
        output << "    mov eax, [ebp + 12]    ; width\n";
        output << "    mov ebx, [ebp + 16]    ; height\n";
        output << "    \n";
        output << "    ; Validate dimensions\n";
        output << "    cmp eax, 0\n";
        output << "    jle .error\n";
        output << "    cmp eax, 32\n";
        output << "    jg .error\n";
        output << "    cmp ebx, 0\n";
        output << "    jle .error\n";
        output << "    cmp ebx, 32\n";
        output << "    jg .error\n";
        output << "    \n";
        output << "    ; Update cursor dimensions\n";
        output << "    mov [mouse_cursor_width], eax\n";
        output << "    mov [mouse_cursor_height], ebx\n";
        output << "    \n";
        output << "    ; Copy sprite data\n";
        output << "    mov edi, mouse_cursor_sprite\n";
        output << "    mov ecx, eax\n";
        output << "    mul ebx\n";
        output << "    mov ecx, eax\n";
        output << "    rep movsb\n";
        output << "    \n";
        output << "    mov eax, 1\n";
        output << "    jmp .done\n";
        output << ".error:\n";
        output << "    xor eax, eax\n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Add mouse event to queue\n";
        output << "; Input: type in eax, x in ebx, y in ecx, buttons in dl\n";
        output << "_mouse_add_event:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov edi, [mouse_event_tail]\n";
        output << "    cmp edi, [mouse_event_head]\n";
        output << "    je .check_full\n";
        output << "    jmp .add_event\n";
        output << ".check_full:\n";
        output << "    ; Check if queue is full\n";
        output << "    mov eax, [mouse_event_head]\n";
        output << "    inc eax\n";
        output << "    cmp eax, " << MOUSE_EVENT_QUEUE_SIZE << "\n";
        output << "    jl .not_wrap\n";
        output << "    xor eax, eax\n";
        output << ".not_wrap:\n";
        output << "    cmp eax, [mouse_event_tail]\n";
        output << "    je .full\n";
        output << "    \n";
        output << ".add_event:\n";
        output << "    mov edi, [mouse_event_tail]\n";
        output << "    imul edi, edi, 12\n";
        output << "    add edi, mouse_event_queue\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]    ; type\n";
        output << "    mov [edi], eax\n";
        output << "    mov eax, [ebp + 12]   ; x\n";
        output << "    mov [edi + 4], eax\n";
        output << "    mov eax, [ebp + 16]   ; y\n";
        output << "    mov [edi + 8], eax\n";
        output << "    mov al, [ebp + 20]    ; buttons\n";
        output << "    mov [edi + 12], al\n";
        output << "    \n";
        output << "    ; Update tail\n";
        output << "    mov edi, [mouse_event_tail]\n";
        output << "    inc edi\n";
        output << "    cmp edi, " << MOUSE_EVENT_QUEUE_SIZE << "\n";
        output << "    jl .tail_no_wrap\n";
        output << "    xor edi, edi\n";
        output << ".tail_no_wrap:\n";
        output << "    mov [mouse_event_tail], edi\n";
        output << ".full:\n";
        output << "    pop edi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Draw mouse cursor at current position\n";
        output << "_mouse_draw_cursor:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    ; Check if cursor is enabled\n";
        output << "    cmp byte [mouse_cursor_enabled], 1\n";
        output << "    jne .done\n";
        output << "    \n";
        output << "    ; Get cursor position\n";
        output << "    mov eax, [mouse_x]\n";
        output << "    mov ebx, [mouse_y]\n";
        output << "    \n";
        output << "    ; Get cursor dimensions\n";
        output << "    mov ecx, [mouse_cursor_width]\n";
        output << "    mov edx, [mouse_cursor_height]\n";
        output << "    \n";
        output << "    ; Draw cursor (simplified - just draw a rectangle for now)\n";
        output << "    ; In full implementation, this would draw the sprite\n";
        output << "    push dword 0xFFFFFF   ; white color\n";
        output << "    push edx\n";
        output << "    push ecx\n";
        output << "    push ebx\n";
        output << "    push eax\n";
        output << "    call _vesa_draw_rect\n";
        output << "    add esp, 20\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Erase mouse cursor (restore background)\n";
        output << "_mouse_erase_cursor:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    ; Check if cursor is enabled\n";
        output << "    cmp byte [mouse_cursor_enabled], 1\n";
        output << "    jne .done\n";
        output << "    \n";
        output << "    ; Get last cursor position\n";
        output << "    mov eax, [mouse_last_x]\n";
        output << "    mov ebx, [mouse_last_y]\n";
        output << "    \n";
        output << "    ; Get cursor dimensions\n";
        output << "    mov ecx, [mouse_cursor_width]\n";
        output << "    mov edx, [mouse_cursor_height]\n";
        output << "    \n";
        output << "    ; Restore background (simplified - clear to black)\n";
        output << "    push dword 0x000000   ; black color\n";
        output << "    push edx\n";
        output << "    push ecx\n";
        output << "    push ebx\n";
        output << "    push eax\n";
        output << "    call _vesa_draw_rect\n";
        output << "    add esp, 20\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Update mouse position and draw cursor\n";
        output << "; Input: dx in eax, dy in ebx\n";
        output << "_mouse_update_position:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    \n";
        output << "    ; Erase old cursor\n";
        output << "    call _mouse_erase_cursor\n";
        output << "    \n";
        output << "    ; Update last position\n";
        output << "    mov eax, [mouse_x]\n";
        output << "    mov [mouse_last_x], eax\n";
        output << "    mov eax, [mouse_y]\n";
        output << "    mov [mouse_last_y], eax\n";
        output << "    \n";
        output << "    ; Apply delta\n";
        output << "    mov eax, [ebp + 8]    ; dx\n";
        output << "    add [mouse_x], eax\n";
        output << "    mov eax, [ebp + 12]   ; dy\n";
        output << "    sub [mouse_y], eax    ; Y is inverted\n";
        output << "    \n";
        output << "    ; Clamp to screen bounds (pixel coordinates)\n";
        output << "    mov eax, [mouse_x]\n";
        output << "    cmp eax, 0\n";
        output << "    jge .check_x_max\n";
        output << "    mov dword [mouse_x], 0\n";
        output << ".check_x_max:\n";
        output << "    cmp eax, [vesa_width]\n";
        output << "    jle .check_y\n";
        output << "    mov eax, [vesa_width]\n";
        output << "    dec eax\n";
        output << "    mov [mouse_x], eax\n";
        output << ".check_y:\n";
        output << "    mov eax, [mouse_y]\n";
        output << "    cmp eax, 0\n";
        output << "    jge .check_y_max\n";
        output << "    mov dword [mouse_y], 0\n";
        output << ".check_y_max:\n";
        output << "    cmp eax, [vesa_height]\n";
        output << "    jle .draw\n";
        output << "    mov eax, [vesa_height]\n";
        output << "    dec eax\n";
        output << "    mov [mouse_y], eax\n";
        output << ".draw:\n";
        output << "    ; Draw new cursor\n";
        output << "    call _mouse_draw_cursor\n";
        output << "    \n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Mouse interrupt handler (IRQ12)\n";
        output << "mouse_handler:\n";
        output << "    pushad\n";
        output << "    push ds\n";
        output << "    push es\n";
        output << "    push fs\n";
        output << "    push gs\n";
        output << "    \n";
        output << "    mov ax, 0x10\n";
        output << "    mov ds, ax\n";
        output << "    mov es, ax\n";
        output << "    \n";
        output << "    ; Read mouse data byte\n";
        output << "    in al, " << MOUSE_PORT_DATA << "\n";
        output << "    \n";
        output << "    ; Process packet based on cycle\n";
        output << "    mov ecx, [mouse_cycle]\n";
        output << "    cmp ecx, 0\n";
        output << "    je .byte0\n";
        output << "    cmp ecx, 1\n";
        output << "    je .byte1\n";
        output << "    cmp ecx, 2\n";
        output << "    je .byte2\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".byte0:\n";
        output << "    ; First byte: buttons and sign bits\n";
        output << "    mov [mouse_packet_buffer], al\n";
        output << "    mov dword [mouse_cycle], 1\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".byte1:\n";
        output << "    ; Second byte: X delta\n";
        output << "    mov [mouse_packet_buffer + 1], al\n";
        output << "    mov dword [mouse_cycle], 2\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".byte2:\n";
        output << "    ; Third byte: Y delta\n";
        output << "    mov [mouse_packet_buffer + 2], al\n";
        output << "    mov dword [mouse_cycle], 0\n";
        output << "    \n";
        output << "    ; Process complete packet\n";
        output << "    movzx eax, byte [mouse_packet_buffer]\n";
        output << "    movzx ebx, byte [mouse_packet_buffer + 1]\n";
        output << "    movzx ecx, byte [mouse_packet_buffer + 2]\n";
        output << "    \n";
        output << "    ; Extract buttons\n";
        output << "    mov edx, eax\n";
        output << "    and edx, 0x07\n";
        output << "    mov [mouse_buttons], dl\n";
        output << "    \n";
        output << "    ; Extract sign bits and convert to signed values\n";
        output << "    test eax, 0x10        ; X overflow?\n";
        output << "    jnz .x_overflow\n";
        output << "    test eax, 0x20        ; Y overflow?\n";
        output << "    jnz .y_overflow\n";
        output << "    \n";
        output << "    ; X delta\n";
        output << "    test eax, 0x40        ; X sign bit\n";
        output << "    jz .x_positive\n";
        output << "    or ebx, 0xFFFFFF00    ; Sign extend\n";
        output << "    jmp .x_done\n";
        output << ".x_positive:\n";
        output << "    and ebx, 0xFF\n";
        output << ".x_done:\n";
        output << "    \n";
        output << "    ; Y delta\n";
        output << "    test eax, 0x80        ; Y sign bit\n";
        output << "    jz .y_positive\n";
        output << "    or ecx, 0xFFFFFF00    ; Sign extend\n";
        output << "    jmp .y_done\n";
        output << ".y_positive:\n";
        output << "    and ecx, 0xFF\n";
        output << ".y_done:\n";
        output << "    \n";
        output << "    ; Store deltas\n";
        output << "    mov [mouse_dx], ebx\n";
        output << "    mov [mouse_dy], ecx\n";
        output << "    \n";
        output << "    ; Update cursor position\n";
        output << "    push ecx\n";
        output << "    push ebx\n";
        output << "    call _mouse_update_position\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Check for button state changes\n";
        output << "    mov al, [mouse_buttons]\n";
        output << "    mov bl, [mouse_last_buttons]\n";
        output << "    mov [mouse_last_buttons], al\n";
        output << "    \n";
        output << "    ; Check for button down events\n";
        output << "    mov cl, al\n";
        output << "    not cl\n";
        output << "    and cl, bl\n";
        output << "    test cl, " << MOUSE_LEFT_BUTTON << "\n";
        output << "    jz .check_right_down\n";
        output << "    ; Left button down event\n";
        output << "    push dword " << MOUSE_LEFT_BUTTON << "\n";
        output << "    push dword [mouse_y]\n";
        output << "    push dword [mouse_x]\n";
        output << "    push dword " << MOUSE_EVENT_BUTTON_DOWN << "\n";
        output << "    call _mouse_add_event\n";
        output << "    add esp, 16\n";
        output << "    ; Check double-click\n";
        output << "    mov eax, [mouse_last_click_time]\n";
        output << "    mov ebx, [mouse_double_click_delay]\n";
        output << "    add eax, ebx\n";
        output << "    cmp eax, [mouse_ticks]\n";
        output << "    jg .check_double_click_distance\n";
        output << "    jmp .reset_click\n";
        output << ".check_double_click_distance:\n";
        output << "    mov eax, [mouse_last_click_x]\n";
        output << "    sub eax, [mouse_x]\n";
        output << "    cmp eax, -10\n";
        output << "    jl .reset_click\n";
        output << "    cmp eax, 10\n";
        output << "    jg .reset_click\n";
        output << "    mov eax, [mouse_last_click_y]\n";
        output << "    sub eax, [mouse_y]\n";
        output << "    cmp eax, -10\n";
        output << "    jl .reset_click\n";
        output << "    cmp eax, 10\n";
        output << "    jg .reset_click\n";
        output << "    ; Double-click detected\n";
        output << "    push dword " << MOUSE_LEFT_BUTTON << "\n";
        output << "    push dword [mouse_y]\n";
        output << "    push dword [mouse_x]\n";
        output << "    push dword " << (MOUSE_EVENT_BUTTON_DOWN + 0x80) << "\n";
        output << "    call _mouse_add_event\n";
        output << "    add esp, 16\n";
        output << ".reset_click:\n";
        output << "    mov eax, [mouse_ticks]\n";
        output << "    mov [mouse_last_click_time], eax\n";
        output << "    mov eax, [mouse_x]\n";
        output << "    mov [mouse_last_click_x], eax\n";
        output << "    mov eax, [mouse_y]\n";
        output << "    mov [mouse_last_click_y], eax\n";
        output << "    \n";
        output << ".check_right_down:\n";
        output << "    test cl, " << MOUSE_RIGHT_BUTTON << "\n";
        output << "    jz .check_middle_down\n";
        output << "    ; Right button down event\n";
        output << "    push dword " << MOUSE_RIGHT_BUTTON << "\n";
        output << "    push dword [mouse_y]\n";
        output << "    push dword [mouse_x]\n";
        output << "    push dword " << MOUSE_EVENT_BUTTON_DOWN << "\n";
        output << "    call _mouse_add_event\n";
        output << "    add esp, 16\n";
        output << ".check_middle_down:\n";
        output << "    test cl, " << MOUSE_MIDDLE_BUTTON << "\n";
        output << "    jz .check_button_up\n";
        output << "    ; Middle button down event\n";
        output << "    push dword " << MOUSE_MIDDLE_BUTTON << "\n";
        output << "    push dword [mouse_y]\n";
        output << "    push dword [mouse_x]\n";
        output << "    push dword " << MOUSE_EVENT_BUTTON_DOWN << "\n";
        output << "    call _mouse_add_event\n";
        output << "    add esp, 16\n";
        output << "    \n";
        output << ".check_button_up:\n";
        output << "    ; Check for button up events\n";
        output << "    mov cl, bl\n";
        output << "    not cl\n";
        output << "    and cl, al\n";
        output << "    test cl, " << MOUSE_LEFT_BUTTON << "\n";
        output << "    jz .check_right_up\n";
        output << "    ; Left button up event\n";
        output << "    push dword " << MOUSE_LEFT_BUTTON << "\n";
        output << "    push dword [mouse_y]\n";
        output << "    push dword [mouse_x]\n";
        output << "    push dword " << MOUSE_EVENT_BUTTON_UP << "\n";
        output << "    call _mouse_add_event\n";
        output << "    add esp, 16\n";
        output << ".check_right_up:\n";
        output << "    test cl, " << MOUSE_RIGHT_BUTTON << "\n";
        output << "    jz .check_middle_up\n";
        output << "    ; Right button up event\n";
        output << "    push dword " << MOUSE_RIGHT_BUTTON << "\n";
        output << "    push dword [mouse_y]\n";
        output << "    push dword [mouse_x]\n";
        output << "    push dword " << MOUSE_EVENT_BUTTON_UP << "\n";
        output << "    call _mouse_add_event\n";
        output << "    add esp, 16\n";
        output << ".check_middle_up:\n";
        output << "    test cl, " << MOUSE_MIDDLE_BUTTON << "\n";
        output << "    jz .check_move\n";
        output << "    ; Middle button up event\n";
        output << "    push dword " << MOUSE_MIDDLE_BUTTON << "\n";
        output << "    push dword [mouse_y]\n";
        output << "    push dword [mouse_x]\n";
        output << "    push dword " << MOUSE_EVENT_BUTTON_UP << "\n";
        output << "    call _mouse_add_event\n";
        output << "    add esp, 16\n";
        output << "    \n";
        output << ".check_move:\n";
        output << "    ; Add move event if there was movement\n";
        output << "    cmp ebx, 0\n";
        output << "    jne .add_move\n";
        output << "    cmp ecx, 0\n";
        output << "    jne .add_move\n";
        output << "    jmp .done_packet\n";
        output << ".add_move:\n";
        output << "    push dword [mouse_buttons]\n";
        output << "    push dword [mouse_y]\n";
        output << "    push dword [mouse_x]\n";
        output << "    push dword " << MOUSE_EVENT_MOVE << "\n";
        output << "    call _mouse_add_event\n";
        output << "    add esp, 16\n";
        output << ".done_packet:\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".x_overflow:\n";
        output << ".y_overflow:\n";
        output << "    ; Handle overflow - reset cycle\n";
        output << "    mov dword [mouse_cycle], 0\n";
        output << "    \n";
        output << ".done:\n";
        output << "    ; Send EOI to PIC\n";
        output << "    mov al, 0x20\n";
        output << "    out 0xA0, al          ; Slave PIC\n";
        output << "    out 0x20, al          ; Master PIC\n";
        output << "    \n";
        output << "    pop gs\n";
        output << "    pop fs\n";
        output << "    pop es\n";
        output << "    pop ds\n";
        output << "    popad\n";
        output << "    iret\n";
        output << "\n";
        
        output << "; Mouse API functions\n";
        output << "_mouse_get_x:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    mov eax, [mouse_x]\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_mouse_get_y:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    mov eax, [mouse_y]\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_mouse_get_buttons:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    movzx eax, byte [mouse_buttons]\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_mouse_get_dx:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    mov eax, [mouse_dx]\n";
        output << "    push eax\n";
        output << "    mov dword [mouse_dx], 0\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_mouse_get_dy:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    mov eax, [mouse_dy]\n";
        output << "    push eax\n";
        output << "    mov dword [mouse_dy], 0\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_mouse_set_position:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    \n";
        output << "    ; Erase old cursor\n";
        output << "    call _mouse_erase_cursor\n";
        output << "    \n";
        output << "    ; Update last position\n";
        output << "    mov eax, [mouse_x]\n";
        output << "    mov [mouse_last_x], eax\n";
        output << "    mov eax, [mouse_y]\n";
        output << "    mov [mouse_last_y], eax\n";
        output << "    \n";
        output << "    ; Set new position\n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    mov [mouse_x], eax\n";
        output << "    mov eax, [ebp + 12]\n";
        output << "    mov [mouse_y], eax\n";
        output << "    \n";
        output << "    ; Draw new cursor\n";
        output << "    call _mouse_draw_cursor\n";
        output << "    \n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Mouse event polling function\n";
        output << "_mouse_poll_event:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    \n";
        output << "    ; Check if queue is empty\n";
        output << "    mov eax, [mouse_event_head]\n";
        output << "    cmp eax, [mouse_event_tail]\n";
        output << "    je .empty\n";
        output << "    \n";
        output << "    ; Get event from queue\n";
        output << "    imul eax, eax, 12\n";
        output << "    add eax, mouse_event_queue\n";
        output << "    mov ebx, [ebp + 8]    ; event structure pointer\n";
        output << "    \n";
        output << "    mov ecx, [eax]\n";
        output << "    mov [ebx], ecx\n";
        output << "    mov ecx, [eax + 4]\n";
        output << "    mov [ebx + 4], ecx\n";
        output << "    mov ecx, [eax + 8]\n";
        output << "    mov [ebx + 8], ecx\n";
        output << "    mov cl, [eax + 12]\n";
        output << "    mov [ebx + 12], cl\n";
        output << "    \n";
        output << "    ; Update head pointer\n";
        output << "    mov eax, [mouse_event_head]\n";
        output << "    inc eax\n";
        output << "    cmp eax, " << MOUSE_EVENT_QUEUE_SIZE << "\n";
        output << "    jl .no_wrap\n";
        output << "    xor eax, eax\n";
        output << ".no_wrap:\n";
        output << "    mov [mouse_event_head], eax\n";
        output << "    \n";
        output << "    mov eax, 1\n";
        output << "    jmp .done\n";
        output << ".empty:\n";
        output << "    xor eax, eax\n";
        output << ".done:\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "section .rodata\n";
        output << "default_cursor_pattern:\n";
        output << "    ; Arrow cursor (16x16) - 0=transparent, 1=white, 2=black\n";
        output << "    db 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1\n";
        output << "    db 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1\n";
        output << "\n";
    }
    
    void generateScheduler() {
        output << "; ============================================================\n";
        output << "; Process Scheduler and PCB Management\n";
        output << "; ============================================================\n\n";
        
        output << "; Process states\n";
        output << "    PROCESS_RUNNING equ 1\n";
        output << "    PROCESS_READY equ 2\n";
        output << "    PROCESS_BLOCKED equ 3\n";
        output << "    PROCESS_ZOMBIE equ 4\n";
        output << "\n";
        
        output << "; PCB offsets\n";
        output << "    PCB_STATE equ 0\n";
        output << "    PCB_PID equ 4\n";
        output << "    PCB_PRIORITY equ 8\n";
        output << "    PCB_SLEEP_TICKS equ 12\n";
        output << "    PCB_ESP equ 16\n";
        output << "    PCB_EIP equ 20\n";
        output << "    PCB_EAX equ 24\n";
        output << "    PCB_EBX equ 28\n";
        output << "    PCB_ECX equ 32\n";
        output << "    PCB_EDX equ 36\n";
        output << "    PCB_ESI equ 40\n";
        output << "    PCB_EDI equ 44\n";
        output << "    PCB_EBP equ 48\n";
        output << "    PCB_PARENT equ 52\n";
        output << "    PCB_NAME equ 56\n";
        output << "    PCB_CHILDREN equ 88  ; 32 + 56 = 88\n";
        output << "\n";
        
        output << "; System ticks counter\n";
        output << "    mouse_ticks dd 0\n";
        output << "\n";
        
        output << "; Get PCB address for PID\n";
        output << "; Input: PID in eax\n";
        output << "; Output: eax = PCB address, edx = index\n";
        output << "_get_pcb:\n";
        output << "    push ebx\n";
        output << "    dec eax               ; PID starts at 1\n";
        output << "    mov edx, eax\n";
        output << "    imul eax, PCB_SIZE\n";
        output << "    add eax, process_table\n";
        output << "    pop ebx\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Create new process\n";
        output << "; Input: entry point in [ebp+8], name in [ebp+12], priority in [ebp+16]\n";
        output << "; Output: eax = PID or -1 on error\n";
        output << "_create_process:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push edi\n";
        output << "    push esi\n";
        output << "    \n";
        output << "    ; Check if we can create more processes\n";
        output << "    cmp dword [process_count], " << MAX_PROCESSES << "\n";
        output << "    jge .error\n";
        output << "    \n";
        output << "    ; Get new PID\n";
        output << "    mov eax, [next_pid]\n";
        output << "    inc dword [next_pid]\n";
        output << "    push eax\n";
        output << "    \n";
        output << "    ; Get PCB address\n";
        output << "    call _get_pcb\n";
        output << "    pop eax\n";
        output << "    mov edi, eax          ; PCB address\n";
        output << "    \n";
        output << "    ; Initialize PCB\n";
        output << "    mov dword [edi + PCB_STATE], PROCESS_READY\n";
        output << "    mov [edi + PCB_PID], eax\n";
        output << "    mov ecx, [ebp + 16]   ; priority\n";
        output << "    mov [edi + PCB_PRIORITY], ecx\n";
        output << "    mov dword [edi + PCB_SLEEP_TICKS], 0\n";
        output << "    \n";
        output << "    ; Set parent PID\n";
        output << "    mov ecx, [current_pid]\n";
        output << "    mov [edi + PCB_PARENT], ecx\n";
        output << "    \n";
        output << "    ; Copy name\n";
        output << "    mov esi, [ebp + 12]\n";
        output << "    lea edi, [edi + PCB_NAME]\n";
        output << "    mov ecx, 32\n";
        output << "    cld\n";
        output << "    rep movsb\n";
        output << "    \n";
        output << "    ; Allocate kernel stack for process\n";
        output << "    push dword 4096\n";
        output << "    call _malloc\n";
        output << "    add esp, 4\n";
        output << "    test eax, eax\n";
        output << "    jz .error\n";
        output << "    \n";
        output << "    ; Setup initial stack frame\n";
        output << "    add eax, 4096 - 4     ; stack top minus 4\n";
        output << "    mov [edi + PCB_ESP], eax\n";
        output << "    \n";
        output << "    ; Set instruction pointer\n";
        output << "    mov ecx, [ebp + 8]\n";
        output << "    mov [edi + PCB_EIP], ecx\n";
        output << "    \n";
        output << "    ; Initialize registers to 0\n";
        output << "    mov dword [edi + PCB_EAX], 0\n";
        output << "    mov dword [edi + PCB_EBX], 0\n";
        output << "    mov dword [edi + PCB_ECX], 0\n";
        output << "    mov dword [edi + PCB_EDX], 0\n";
        output << "    mov dword [edi + PCB_ESI], 0\n";
        output << "    mov dword [edi + PCB_EDI], 0\n";
        output << "    mov dword [edi + PCB_EBP], 0\n";
        output << "    \n";
        output << "    ; Add to children list of parent\n";
        output << "    push eax\n";
        output << "    mov eax, [current_pid]\n";
        output << "    call _get_pcb\n";
        output << "    ; Find free slot in children list\n";
        output << "    mov ecx, 64\n";
        output << "    lea edi, [eax + PCB_CHILDREN]\n";
        output << ".find_child_slot:\n";
        output << "    cmp dword [edi], 0\n";
        output << "    je .found_slot\n";
        output << "    add edi, 4\n";
        output << "    loop .find_child_slot\n";
        output << "    pop eax\n";
        output << "    jmp .error\n";
        output << ".found_slot:\n";
        output << "    pop ecx\n";
        output << "    mov [edi], ecx\n";
        output << "    \n";
        output << "    ; Increment process count\n";
        output << "    inc dword [process_count]\n";
        output << "    \n";
        output << "    ; Return PID\n";
        output << "    mov eax, [edi + PCB_PID - PCB_NAME + 56]  ; Get PID back\n";
        output << "    jmp .done\n";
        output << ".error:\n";
        output << "    mov eax, -1\n";
        output << ".done:\n";
        output << "    pop esi\n";
        output << "    pop edi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Create thread (lightweight process)\n";
        output << "_create_thread:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push edi\n";
        output << "    push esi\n";
        output << "    \n";
        output << "    ; Check if we can create more processes\n";
        output << "    cmp dword [process_count], " << MAX_PROCESSES << "\n";
        output << "    jge .error\n";
        output << "    \n";
        output << "    ; Get new PID (TID)\n";
        output << "    mov eax, [next_pid]\n";
        output << "    inc dword [next_pid]\n";
        output << "    push eax\n";
        output << "    \n";
        output << "    ; Get PCB address\n";
        output << "    call _get_pcb\n";
        output << "    pop eax\n";
        output << "    mov edi, eax\n";
        output << "    \n";
        output << "    ; Initialize PCB\n";
        output << "    mov dword [edi + PCB_STATE], PROCESS_READY\n";
        output << "    mov [edi + PCB_PID], eax\n";
        output << "    mov ecx, [ebp + 16]   ; priority\n";
        output << "    mov [edi + PCB_PRIORITY], ecx\n";
        output << "    mov dword [edi + PCB_SLEEP_TICKS], 0\n";
        output << "    \n";
        output << "    ; Set parent PID (thread shares address space with parent)\n";
        output << "    mov ecx, [ebp + 12]   ; parent PID\n";
        output << "    mov [edi + PCB_PARENT], ecx\n";
        output << "    \n";
        output << "    ; Copy name (as thread name)\n";
        output << "    mov esi, [ebp + 12]\n";
        output << "    lea edi, [edi + PCB_NAME]\n";
        output << "    mov ecx, 32\n";
        output << "    cld\n";
        output << "    rep movsb\n";
        output << "    \n";
        output << "    ; Allocate kernel stack for thread\n";
        output << "    push dword 4096\n";
        output << "    call _malloc\n";
        output << "    add esp, 4\n";
        output << "    test eax, eax\n";
        output << "    jz .error\n";
        output << "    \n";
        output << "    ; Setup initial stack frame\n";
        output << "    add eax, 4096 - 4\n";
        output << "    mov [edi + PCB_ESP], eax\n";
        output << "    \n";
        output << "    ; Set instruction pointer\n";
        output << "    mov ecx, [ebp + 8]\n";
        output << "    mov [edi + PCB_EIP], ecx\n";
        output << "    \n";
        output << "    ; Initialize registers to 0\n";
        output << "    mov dword [edi + PCB_EAX], 0\n";
        output << "    mov dword [edi + PCB_EBX], 0\n";
        output << "    mov dword [edi + PCB_ECX], 0\n";
        output << "    mov dword [edi + PCB_EDX], 0\n";
        output << "    mov dword [edi + PCB_ESI], 0\n";
        output << "    mov dword [edi + PCB_EDI], 0\n";
        output << "    mov dword [edi + PCB_EBP], 0\n";
        output << "    \n";
        output << "    ; Add to children list of parent\n";
        output << "    push eax\n";
        output << "    mov eax, [ebp + 12]\n";
        output << "    call _get_pcb\n";
        output << "    ; Find free slot in children list\n";
        output << "    mov ecx, 64\n";
        output << "    lea edi, [eax + PCB_CHILDREN]\n";
        output << ".find_child_slot:\n";
        output << "    cmp dword [edi], 0\n";
        output << "    je .found_slot\n";
        output << "    add edi, 4\n";
        output << "    loop .find_child_slot\n";
        output << "    pop eax\n";
        output << "    jmp .error\n";
        output << ".found_slot:\n";
        output << "    pop ecx\n";
        output << "    mov [edi], ecx\n";
        output << "    \n";
        output << "    ; Increment process count\n";
        output << "    inc dword [process_count]\n";
        output << "    \n";
        output << "    ; Return PID\n";
        output << "    mov eax, [edi + PCB_PID - PCB_NAME + 56]\n";
        output << "    jmp .done\n";
        output << ".error:\n";
        output << "    mov eax, -1\n";
        output << ".done:\n";
        output << "    pop esi\n";
        output << "    pop edi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Yield - voluntarily give up CPU\n";
        output << "_yield:\n";
        output << "    mov byte [need_schedule], 1\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Sleep for milliseconds\n";
        output << "_sleep:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]    ; milliseconds\n";
        output << "    mov ecx, " << PIT_FREQUENCY << "\n";
        output << "    div ecx               ; ticks = ms / (1000 / freq)\n";
        output << "    \n";
        output << "    ; Get current PCB\n";
        output << "    push eax\n";
        output << "    mov eax, [current_pid]\n";
        output << "    call _get_pcb\n";
        output << "    pop ecx\n";
        output << "    \n";
        output << "    ; Set sleep ticks\n";
        output << "    mov [eax + PCB_SLEEP_TICKS], ecx\n";
        output << "    \n";
        output << "    ; Change state to BLOCKED\n";
        output << "    mov dword [eax + PCB_STATE], PROCESS_BLOCKED\n";
        output << "    \n";
        output << "    ; Force schedule\n";
        output << "    mov byte [need_schedule], 1\n";
        output << "    \n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Wait for process to exit\n";
        output << "_wait_pid:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]    ; PID to wait for\n";
        output << "    call _get_pcb\n";
        output << "    mov ebx, eax\n";
        output << "    \n";
        output << ".wait_loop:\n";
        output << "    cmp dword [ebx + PCB_STATE], PROCESS_ZOMBIE\n";
        output << "    je .done\n";
        output << "    \n";
        output << "    ; Force schedule\n";
        output << "    mov byte [need_schedule], 1\n";
        output << "    \n";
        output << "    ; Short delay\n";
        output << "    mov ecx, 100\n";
        output << ".delay:\n";
        output << "    loop .delay\n";
        output << "    \n";
        output << "    jmp .wait_loop\n";
        output << ".done:\n";
        output << "    ; Free PCB and stack\n";
        output << "    mov eax, [ebx + PCB_ESP]\n";
        output << "    sub eax, 4095\n";
        output << "    push eax\n";
        output << "    call _free\n";
        output << "    add esp, 4\n";
        output << "    \n";
        output << "    mov dword [ebx + PCB_STATE], 0\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Kill process\n";
        output << "_kill_pid:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]    ; PID to kill\n";
        output << "    call _get_pcb\n";
        output << "    mov ebx, eax\n";
        output << "    \n";
        output << "    ; Mark as zombie\n";
        output << "    mov dword [ebx + PCB_STATE], PROCESS_ZOMBIE\n";
        output << "    \n";
        output << "    ; Force schedule\n";
        output << "    mov byte [need_schedule], 1\n";
        output << "    \n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Get current process ID\n";
        output << "_getpid:\n";
        output << "    mov eax, [current_pid]\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Get parent process ID\n";
        output << "_getppid:\n";
        output << "    push eax\n";
        output << "    mov eax, [current_pid]\n";
        output << "    call _get_pcb\n";
        output << "    mov eax, [eax + PCB_PARENT]\n";
        output << "    pop ecx\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Set process priority\n";
        output << "_set_priority:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]    ; PID\n";
        output << "    call _get_pcb\n";
        output << "    mov ecx, [ebp + 12]   ; priority\n";
        output << "    mov [eax + PCB_PRIORITY], ecx\n";
        output << "    \n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Context switch - save current process state\n";
        output << "_save_state:\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    \n";
        output << "    ; Get current PCB\n";
        output << "    mov eax, [current_pid]\n";
        output << "    call _get_pcb\n";
        output << "    mov ebx, eax\n";
        output << "    \n";
        output << "    ; Save registers\n";
        output << "    pop eax               ; restore eax after push\n";
        output << "    mov [ebx + PCB_EAX], eax\n";
        output << "    pop eax\n";
        output << "    mov [ebx + PCB_EBX], eax\n";
        output << "    pop eax\n";
        output << "    mov [ebx + PCB_ECX], eax\n";
        output << "    pop eax\n";
        output << "    mov [ebx + PCB_EDX], eax\n";
        output << "    pop eax\n";
        output << "    mov [ebx + PCB_ESI], eax\n";
        output << "    pop eax\n";
        output << "    mov [ebx + PCB_EDI], eax\n";
        output << "    pop eax\n";
        output << "    mov [ebx + PCB_EBP], eax\n";
        output << "    \n";
        output << "    ; Save return address as EIP\n";
        output << "    pop eax\n";
        output << "    mov [ebx + PCB_EIP], eax\n";
        output << "    \n";
        output << "    ; Save stack pointer\n";
        output << "    mov [ebx + PCB_ESP], esp\n";
        output << "    \n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Restore process state\n";
        output << "_restore_state:\n";
        output << "    ; Get PCB from eax\n";
        output << "    mov ebx, eax\n";
        output << "    \n";
        output << "    ; Restore stack pointer\n";
        output << "    mov esp, [ebx + PCB_ESP]\n";
        output << "    \n";
        output << "    ; Push registers in reverse order\n";
        output << "    push dword [ebx + PCB_EBP]\n";
        output << "    push dword [ebx + PCB_EDI]\n";
        output << "    push dword [ebx + PCB_ESI]\n";
        output << "    push dword [ebx + PCB_EDX]\n";
        output << "    push dword [ebx + PCB_ECX]\n";
        output << "    push dword [ebx + PCB_EBX]\n";
        output << "    push dword [ebx + PCB_EAX]\n";
        output << "    \n";
        output << "    ; Push return address\n";
        output << "    push dword [ebx + PCB_EIP]\n";
        output << "    \n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Scheduler - choose next process to run\n";
        output << "_schedule:\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    ; Clear need_schedule flag\n";
        output << "    mov byte [need_schedule], 0\n";
        output << "    \n";
        output << "    ; Find next ready process with highest priority\n";
        output << "    mov esi, process_table\n";
        output << "    mov ecx, " << MAX_PROCESSES << "\n";
        output << "    xor edx, edx          ; best priority found\n";
        output << "    xor eax, eax          ; best PCB address\n";
        output << "    \n";
        output << ".find_next:\n";
        output << "    cmp dword [esi + PCB_STATE], PROCESS_READY\n";
        output << "    jne .skip\n";
        output << "    \n";
        output << "    mov ebx, [esi + PCB_PRIORITY]\n";
        output << "    cmp ebx, edx\n";
        output << "    jle .skip\n";
        output << "    \n";
        output << "    mov edx, ebx\n";
        output << "    mov eax, esi\n";
        output << "    \n";
        output << ".skip:\n";
        output << "    add esi, PCB_SIZE\n";
        output << "    loop .find_next\n";
        output << "    \n";
        output << "    test eax, eax\n";
        output << "    jnz .found\n";
        output << "    \n";
        output << "    ; No ready process found, idle\n";
        output << "    mov eax, 0\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".found:\n";
        output << "    ; Get PID from PCB\n";
        output << "    mov ebx, [eax + PCB_PID]\n";
        output << "    \n";
        output << "    ; Mark as running\n";
        output << "    mov dword [eax + PCB_STATE], PROCESS_RUNNING\n";
        output << "    \n";
        output << "    ; Update current PID\n";
        output << "    mov [current_pid], ebx\n";
        output << "    \n";
        output << "    ; Reset quantum counter\n";
        output << "    mov dword [quantum_counter], " << QUANTUM_MS << " * " << PIT_FREQUENCY << " / 1000\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Switch to process\n";
        output << "; Input: PCB address in eax\n";
        output << "_switch_to:\n";
        output << "    call _restore_state\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Timer interrupt handler (IRQ0)\n";
        output << "timer_handler:\n";
        output << "    pushad\n";
        output << "    push ds\n";
        output << "    push es\n";
        output << "    push fs\n";
        output << "    push gs\n";
        output << "    \n";
        output << "    mov ax, 0x10\n";
        output << "    mov ds, ax\n";
        output << "    mov es, ax\n";
        output << "    \n";
        output << "    ; Increment system ticks\n";
        output << "    inc dword [mouse_ticks]\n";
        output << "    \n";
        output << "    ; Decrement quantum counter\n";
        output << "    dec dword [quantum_counter]\n";
        output << "    \n";
        output << "    ; Check if quantum expired\n";
        output << "    cmp dword [quantum_counter], 0\n";
        output << "    jle .schedule\n";
        output << "    \n";
        output << "    ; Decrement sleep ticks for blocked processes\n";
        output << "    mov esi, process_table\n";
        output << "    mov ecx, " << MAX_PROCESSES << "\n";
        output << ".sleep_loop:\n";
        output << "    cmp dword [esi + PCB_STATE], PROCESS_BLOCKED\n";
        output << "    jne .next_sleep\n";
        output << "    \n";
        output << "    dec dword [esi + PCB_SLEEP_TICKS]\n";
        output << "    cmp dword [esi + PCB_SLEEP_TICKS], 0\n";
        output << "    jg .next_sleep\n";
        output << "    \n";
        output << "    ; Wake up process\n";
        output << "    mov dword [esi + PCB_STATE], PROCESS_READY\n";
        output << "    mov byte [need_schedule], 1\n";
        output << "    \n";
        output << ".next_sleep:\n";
        output << "    add esi, PCB_SIZE\n";
        output << "    loop .sleep_loop\n";
        output << "    \n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".schedule:\n";
        output << "    mov byte [need_schedule], 1\n";
        output << "    \n";
        output << ".done:\n";
        output << "    ; Send EOI to PIC\n";
        output << "    mov al, 0x20\n";
        output << "    out 0x20, al\n";
        output << "    \n";
        output << "    pop gs\n";
        output << "    pop fs\n";
        output << "    pop es\n";
        output << "    pop ds\n";
        output << "    popad\n";
        output << "    iret\n";
        output << "\n";
        
        output << "; Initialize PIT (Programmable Interval Timer)\n";
        output << "_init_pit:\n";
        output << "    push eax\n";
        output << "    push edx\n";
        output << "    \n";
        output << "    ; Calculate divisor for 100Hz\n";
        output << "    mov eax, 1193180      ; PIT frequency\n";
        output << "    mov edx, " << PIT_FREQUENCY << "\n";
        output << "    div edx               ; eax = divisor\n";
        output << "    \n";
        output << "    ; Set PIT channel 0\n";
        output << "    mov al, PIT_MODE\n";
        output << "    out PIT_COMMAND, al\n";
        output << "    \n";
        output << "    ; Set low byte\n";
        output << "    mov al, byte [eax]\n";
        output << "    out PIT_CHANNEL0, al\n";
        output << "    \n";
        output << "    ; Set high byte\n";
        output << "    shr eax, 8\n";
        output << "    out PIT_CHANNEL0, al\n";
        output << "    \n";
        output << "    pop edx\n";
        output << "    pop eax\n";
        output << "    ret\n";
        output << "\n";
    }
    
    void generateHeapManagement() {
        output << "; ============================================================\n";
        output << "; Heap Management Functions\n";
        output << "; ============================================================\n\n";
        
        output << "; Initialize heap\n";
        output << "_heap_init:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    \n";
        output << "    ; Create initial free block\n";
        output << "    mov eax, [heap_start]\n";
        output << "    mov [free_list], eax\n";
        output << "    \n";
        output << "    ; Set block size (total heap size)\n";
        output << "    mov ebx, [heap_end]\n";
        output << "    sub ebx, eax\n";
        output << "    mov [eax], ebx        ; size field\n";
        output << "    mov dword [eax + 4], 0 ; next pointer (end of list)\n";
        output << "    \n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; malloc - allocate memory\n";
        output << "; Input: size in [ebp+8]\n";
        output << "; Output: eax = pointer to allocated memory (or 0 on failure)\n";
        output << "_malloc:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    \n";
        output << "    mov ecx, [ebp + 8]    ; requested size\n";
        output << "    add ecx, 8            ; Add header size\n";
        output << "    \n";
        output << "    mov esi, [free_list]  ; current free block\n";
        output << "    mov edx, 0            ; previous block pointer\n";
        output << "    \n";
        output << ".find_fit:\n";
        output << "    test esi, esi\n";
        output << "    jz .no_memory\n";
        output << "    \n";
        output << "    mov eax, [esi]        ; block size\n";
        output << "    cmp eax, ecx\n";
        output << "    jge .found_block\n";
        output << "    \n";
        output << "    ; Move to next block\n";
        output << "    mov edx, esi\n";
        output << "    mov esi, [esi + 4]\n";
        output << "    jmp .find_fit\n";
        output << "    \n";
        output << ".found_block:\n";
        output << "    ; Check if block can be split\n";
        output << "    mov ebx, eax\n";
        output << "    sub ebx, ecx\n";
        output << "    cmp ebx, 16          ; Minimum block size for splitting\n";
        output << "    jl .use_full_block\n";
        output << "    \n";
        output << "    ; Split the block\n";
        output << "    mov eax, esi\n";
        output << "    add eax, ecx         ; new block address\n";
        output << "    mov [eax], ebx       ; set size of new free block\n";
        output << "    mov ebx, [esi + 4]   ; get next pointer\n";
        output << "    mov [eax + 4], ebx   ; set next pointer for new free block\n";
        output << "    \n";
        output << "    ; Update previous block's next pointer\n";
        output << "    test edx, edx\n";
        output << "    jz .update_head\n";
        output << "    mov [edx + 4], eax   ; previous->next = new free block\n";
        output << "    jmp .block_allocated\n";
        output << ".update_head:\n";
        output << "    mov [free_list], eax\n";
        output << "    jmp .block_allocated\n";
        output << "    \n";
        output << ".use_full_block:\n";
        output << "    ; Remove block from free list\n";
        output << "    mov eax, [esi + 4]   ; next pointer\n";
        output << "    test edx, edx\n";
        output << "    jz .remove_head\n";
        output << "    mov [edx + 4], eax   ; previous->next = next\n";
        output << "    jmp .block_allocated\n";
        output << ".remove_head:\n";
        output << "    mov [free_list], eax\n";
        output << "    \n";
        output << ".block_allocated:\n";
        output << "    ; Return pointer to user data (skip header)\n";
        output << "    mov eax, esi\n";
        output << "    add eax, 8\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "    \n";
        output << ".no_memory:\n";
        output << "    xor eax, eax\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; free - free allocated memory\n";
        output << "; Input: ptr in [ebp+8]\n";
        output << "_free:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]    ; user pointer\n";
        output << "    test eax, eax\n";
        output << "    jz .done\n";
        output << "    \n";
        output << "    sub eax, 8            ; get block header address\n";
        output << "    mov ebx, [eax]        ; block size\n";
        output << "    \n";
        output << "    ; Insert into free list (sorted by address)\n";
        output << "    mov esi, [free_list]\n";
        output << "    mov edx, 0            ; previous\n";
        output << "    \n";
        output << ".find_position:\n";
        output << "    test esi, esi\n";
        output << "    jz .insert_at_end\n";
        output << "    cmp eax, esi\n";
        output << "    jb .insert_before\n";
        output << "    \n";
        output << "    mov edx, esi\n";
        output << "    mov esi, [esi + 4]\n";
        output << "    jmp .find_position\n";
        output << "    \n";
        output << ".insert_before:\n";
        output << "    test edx, edx\n";
        output << "    jz .insert_at_head\n";
        output << "    mov [edx + 4], eax\n";
        output << "    jmp .set_next\n";
        output << ".insert_at_head:\n";
        output << "    mov [free_list], eax\n";
        output << ".set_next:\n";
        output << "    mov [eax + 4], esi\n";
        output << "    jmp .coalesce\n";
        output << "    \n";
        output << ".insert_at_end:\n";
        output << "    test edx, edx\n";
        output << "    jz .set_as_head\n";
        output << "    mov [edx + 4], eax\n";
        output << "    mov dword [eax + 4], 0\n";
        output << "    jmp .coalesce\n";
        output << ".set_as_head:\n";
        output << "    mov [free_list], eax\n";
        output << "    mov dword [eax + 4], 0\n";
        output << "    \n";
        output << ".coalesce:\n";
        output << "    ; Try to merge with next block\n";
        output << "    mov esi, [eax + 4]\n";
        output << "    test esi, esi\n";
        output << "    jz .check_previous\n";
        output << "    \n";
        output << "    mov ecx, eax\n";
        output << "    add ecx, [eax]        ; end of current block\n";
        output << "    cmp ecx, esi\n";
        output << "    jne .check_previous\n";
        output << "    \n";
        output << "    ; Merge with next\n";
        output << "    mov ecx, [esi]        ; next block size\n";
        output << "    add [eax], ecx\n";
        output << "    mov ecx, [esi + 4]    ; next->next\n";
        output << "    mov [eax + 4], ecx\n";
        output << "    \n";
        output << ".check_previous:\n";
        output << "    ; Try to merge with previous block\n";
        output << "    ; Find previous block in free list\n";
        output << "    mov esi, [free_list]\n";
        output << "    mov edx, 0\n";
        output << ".find_prev:\n";
        output << "    test esi, esi\n";
        output << "    jz .done\n";
        output << "    cmp esi, eax\n";
        output << "    je .done\n";
        output << "    \n";
        output << "    mov edx, esi\n";
        output << "    mov esi, [esi + 4]\n";
        output << "    jmp .find_prev\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; calloc - allocate zero-initialized memory\n";
        output << "_calloc:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]    ; count\n";
        output << "    mov ebx, [ebp + 12]   ; size\n";
        output << "    mul ebx               ; total size = count * size\n";
        output << "    \n";
        output << "    push eax\n";
        output << "    call _malloc\n";
        output << "    add esp, 4\n";
        output << "    \n";
        output << "    test eax, eax\n";
        output << "    jz .done\n";
        output << "    \n";
        output << "    ; Zero out memory\n";
        output << "    mov edi, eax\n";
        output << "    mov ecx, [ebp + 8]\n";
        output << "    mov ebx, [ebp + 12]\n";
        output << "    mul ebx\n";
        output << "    mov ecx, eax\n";
        output << "    xor eax, eax\n";
        output << "    rep stosb\n";
        output << "    \n";
        output << "    mov eax, edi\n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; realloc - resize allocated memory\n";
        output << "_realloc:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edi\n";
        output << "    push esi\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]    ; old pointer\n";
        output << "    test eax, eax\n";
        output << "    jz .malloc_only\n";
        output << "    \n";
        output << "    mov ebx, [ebp + 12]   ; new size\n";
        output << "    test ebx, ebx\n";
        output << "    jz .free_only\n";
        output << "    \n";
        output << "    ; Get old block size\n";
        output << "    mov edi, eax\n";
        output << "    sub edi, 8            ; block header\n";
        output << "    mov ecx, [edi]        ; old block size\n";
        output << "    sub ecx, 8            ; actual data size\n";
        output << "    \n";
        output << "    cmp ebx, ecx\n";
        output << "    jle .no_shrink        ; new size <= old size, no need to reallocate\n";
        output << "    \n";
        output << "    ; Allocate new block\n";
        output << "    push ebx\n";
        output << "    call _malloc\n";
        output << "    add esp, 4\n";
        output << "    \n";
        output << "    test eax, eax\n";
        output << "    jz .realloc_fail\n";
        output << "    \n";
        output << "    ; Copy old data to new block\n";
        output << "    mov esi, [ebp + 8]    ; source\n";
        output << "    mov edi, eax          ; destination\n";
        output << "    mov ecx, ebx\n";
        output << "    cmp ecx, [ebp + 8]\n";
        output << "    jle .copy\n";
        output << "    mov ecx, [ebp + 8]\n";
        output << ".copy:\n";
        output << "    rep movsb\n";
        output << "    \n";
        output << "    ; Free old block\n";
        output << "    push dword [ebp + 8]\n";
        output << "    call _free\n";
        output << "    add esp, 4\n";
        output << "    \n";
        output << "    jmp .realloc_done\n";
        output << "    \n";
        output << ".no_shrink:\n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    jmp .realloc_done\n";
        output << "    \n";
        output << ".malloc_only:\n";
        output << "    push dword [ebp + 12]\n";
        output << "    call _malloc\n";
        output << "    add esp, 4\n";
        output << "    jmp .realloc_done\n";
        output << "    \n";
        output << ".free_only:\n";
        output << "    push dword [ebp + 8]\n";
        output << "    call _free\n";
        output << "    add esp, 4\n";
        output << "    xor eax, eax\n";
        output << "    jmp .realloc_done\n";
        output << "    \n";
        output << ".realloc_fail:\n";
        output << "    xor eax, eax\n";
        output << ".realloc_done:\n";
        output << "    pop esi\n";
        output << "    pop edi\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; heap_used - get total used memory\n";
        output << "_heap_used:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push esi\n";
        output << "    \n";
        output << "    mov esi, [free_list]\n";
        output << "    mov eax, [heap_end]\n";
        output << "    sub eax, [heap_start]\n";
        output << "    mov ecx, eax          ; total heap size\n";
        output << "    xor eax, eax\n";
        output << "    \n";
        output << ".sum_free:\n";
        output << "    test esi, esi\n";
        output << "    jz .done\n";
        output << "    add eax, [esi]        ; add free block size\n";
        output << "    mov esi, [esi + 4]\n";
        output << "    jmp .sum_free\n";
        output << "    \n";
        output << ".done:\n";
        output << "    sub ecx, eax          ; used = total - free\n";
        output << "    mov eax, ecx\n";
        output << "    pop esi\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; heap_free - get total free memory\n";
        output << "_heap_free:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push esi\n";
        output << "    \n";
        output << "    mov esi, [free_list]\n";
        output << "    xor eax, eax\n";
        output << "    \n";
        output << ".sum_free:\n";
        output << "    test esi, esi\n";
        output << "    jz .done\n";
        output << "    add eax, [esi]\n";
        output << "    mov esi, [esi + 4]\n";
        output << "    jmp .sum_free\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop esi\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; heap_dump - debug function to print heap info\n";
        output << "_heap_dump:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push esi\n";
        output << "    \n";
        output << "    push str_heap_dump\n";
        output << "    call print_string\n";
        output << "    add esp, 4\n";
        output << "    \n";
        output << "    mov esi, [free_list]\n";
        output << "    mov ecx, 0\n";
        output << ".loop:\n";
        output << "    test esi, esi\n";
        output << "    jz .done\n";
        output << "    push esi\n";
        output << "    push ecx\n";
        output << "    push str_block\n";
        output << "    call print_string\n";
        output << "    add esp, 4\n";
        output << "    pop ecx\n";
        output << "    pop esi\n";
        output << "    \n";
        output << "    push dword [esi]      ; block size\n";
        output << "    call print_number\n";
        output << "    add esp, 4\n";
        output << "    \n";
        output << "    push str_newline\n";
        output << "    call print_string\n";
        output << "    add esp, 4\n";
        output << "    \n";
        output << "    mov esi, [esi + 4]\n";
        output << "    inc ecx\n";
        output << "    jmp .loop\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop esi\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "section .rodata\n";
        output << "str_heap_dump db `\\n=== HEAP DUMP ===\\n`, 0\n";
        output << "str_block db `  Block size: `, 0\n";
        output << "str_newline db `\\n`, 0\n";
        output << "\n";
    }
    
    void generateDiskIO() {
        output << "; ============================================================\n";
        output << "; Disk I/O Functions (ATA PIO)\n";
        output << "; ============================================================\n\n";
        
        output << "; Wait for disk ready\n";
        output << "_wait_disk_ready:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ecx\n";
        output << "    \n";
        output << "    mov ecx, 100000      ; timeout counter\n";
        output << ".wait_loop:\n";
        output << "    mov dx, IDE_STATUS_PORT\n";
        output << "    in al, dx\n";
        output << "    and al, 0xC0         ; Check BSY and DRQ\n";
        output << "    cmp al, 0x40         ; DRQ set, BSY clear\n";
        output << "    je .ready\n";
        output << "    loop .wait_loop\n";
        output << "    \n";
        output << ".ready:\n";
        output << "    pop ecx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Read sector from disk\n";
        output << "; Input: LBA in [ebp+8], buffer in [ebp+12]\n";
        output << "; Output: eax = 1 on success, 0 on failure\n";
        output << "_read_sector:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]   ; LBA\n";
        output << "    mov esi, [ebp + 12]  ; buffer\n";
        output << "    \n";
        output << "    ; Wait for disk ready\n";
        output << "    call _wait_disk_ready\n";
        output << "    \n";
        output << "    ; Select drive\n";
        output << "    mov dx, IDE_DRIVE_HEAD_PORT\n";
        output << "    mov cl, 0xE0         ; LBA mode, master drive\n";
        output << "    and cl, 0x0F\n";
        output << "    mov ch, al\n";
        output << "    shr ch, 24\n";
        output << "    and ch, 0x0F\n";
        output << "    or cl, ch\n";
        output << "    mov al, cl\n";
        output << "    out dx, al\n";
        output << "    \n";
        output << "    ; Set sector count\n";
        output << "    mov dx, IDE_SECTOR_COUNT_PORT\n";
        output << "    mov al, 1\n";
        output << "    out dx, al\n";
        output << "    \n";
        output << "    ; Set LBA low\n";
        output << "    mov dx, IDE_SECTOR_NUMBER_PORT\n";
        output << "    mov al, byte [ebp + 8]\n";
        output << "    out dx, al\n";
        output << "    \n";
        output << "    ; Set LBA mid\n";
        output << "    mov dx, IDE_CYLINDER_LOW_PORT\n";
        output << "    mov al, byte [ebp + 9]\n";
        output << "    out dx, al\n";
        output << "    \n";
        output << "    ; Set LBA high\n";
        output << "    mov dx, IDE_CYLINDER_HIGH_PORT\n";
        output << "    mov al, byte [ebp + 10]\n";
        output << "    out dx, al\n";
        output << "    \n";
        output << "    ; Send read command\n";
        output << "    mov dx, IDE_COMMAND_PORT\n";
        output << "    mov al, ATA_CMD_READ_SECTORS\n";
        output << "    out dx, al\n";
        output << "    \n";
        output << "    ; Wait for disk ready\n";
        output << "    call _wait_disk_ready\n";
        output << "    \n";
        output << "    ; Read data\n";
        output << "    mov dx, IDE_DATA_PORT\n";
        output << "    mov ecx, 256          ; 256 words = 512 bytes\n";
        output << "    cld\n";
        output << ".read_loop:\n";
        output << "    in ax, dx\n";
        output << "    mov [esi], ax\n";
        output << "    add esi, 2\n";
        output << "    loop .read_loop\n";
        output << "    \n";
        output << "    mov eax, 1\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Write sector to disk\n";
        output << "; Input: LBA in [ebp+8], buffer in [ebp+12]\n";
        output << "; Output: eax = 1 on success, 0 on failure\n";
        output << "_write_sector:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]   ; LBA\n";
        output << "    mov esi, [ebp + 12]  ; buffer\n";
        output << "    \n";
        output << "    ; Wait for disk ready\n";
        output << "    call _wait_disk_ready\n";
        output << "    \n";
        output << "    ; Select drive\n";
        output << "    mov dx, IDE_DRIVE_HEAD_PORT\n";
        output << "    mov cl, 0xE0         ; LBA mode, master drive\n";
        output << "    and cl, 0x0F\n";
        output << "    mov ch, al\n";
        output << "    shr ch, 24\n";
        output << "    and ch, 0x0F\n";
        output << "    or cl, ch\n";
        output << "    mov al, cl\n";
        output << "    out dx, al\n";
        output << "    \n";
        output << "    ; Set sector count\n";
        output << "    mov dx, IDE_SECTOR_COUNT_PORT\n";
        output << "    mov al, 1\n";
        output << "    out dx, al\n";
        output << "    \n";
        output << "    ; Set LBA low\n";
        output << "    mov dx, IDE_SECTOR_NUMBER_PORT\n";
        output << "    mov al, byte [ebp + 8]\n";
        output << "    out dx, al\n";
        output << "    \n";
        output << "    ; Set LBA mid\n";
        output << "    mov dx, IDE_CYLINDER_LOW_PORT\n";
        output << "    mov al, byte [ebp + 9]\n";
        output << "    out dx, al\n";
        output << "    \n";
        output << "    ; Set LBA high\n";
        output << "    mov dx, IDE_CYLINDER_HIGH_PORT\n";
        output << "    mov al, byte [ebp + 10]\n";
        output << "    out dx, al\n";
        output << "    \n";
        output << "    ; Send write command\n";
        output << "    mov dx, IDE_COMMAND_PORT\n";
        output << "    mov al, ATA_CMD_WRITE_SECTORS\n";
        output << "    out dx, al\n";
        output << "    \n";
        output << "    ; Wait for disk ready\n";
        output << "    call _wait_disk_ready\n";
        output << "    \n";
        output << "    ; Write data\n";
        output << "    mov dx, IDE_DATA_PORT\n";
        output << "    mov ecx, 256          ; 256 words = 512 bytes\n";
        output << "    cld\n";
        output << ".write_loop:\n";
        output << "    mov ax, [esi]\n";
        output << "    out dx, ax\n";
        output << "    add esi, 2\n";
        output << "    loop .write_loop\n";
        output << "    \n";
        output << "    mov eax, 1\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
    }
    
    void generateEXT2FileSystem() {
        output << "; ============================================================\n";
        output << "; EXT2/EXT3/EXT4 File System Functions\n";
        output << "; ============================================================\n\n";
        
        output << "; EXT2 block cache functions\n";
        output << "_ext2_read_block:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov ebx, [ebp + 8]    ; block number\n";
        output << "    mov edi, [ebp + 12]   ; buffer pointer\n";
        output << "    \n";
        output << "    ; Check cache\n";
        output << "    mov ecx, " << EXT2_BLOCK_CACHE_SIZE << "\n";
        output << "    mov esi, ext2_cache_block_numbers\n";
        output << "    xor edx, edx\n";
        output << ".check_cache:\n";
        output << "    cmp [esi], ebx\n";
        output << "    je .cache_hit\n";
        output << "    add esi, 4\n";
        output << "    inc edx\n";
        output << "    loop .check_cache\n";
        output << "    \n";
        output << "    ; Cache miss - need to read from disk\n";
        output << "    ; Calculate LBA from block number\n";
        output << "    mov eax, [ext2_block_size]\n";
        output << "    mov ecx, " << SECTOR_SIZE << "\n";
        output << "    div ecx               ; blocks per sector = block_size / 512\n";
        output << "    mov esi, eax          ; sectors per block\n";
        output << "    \n";
        output << "    mov eax, ebx\n";
        output << "    mul esi               ; start sector = block * sectors_per_block\n";
        output << "    add eax, [ext2_device_start_lba]\n";
        output << "    mov ebx, eax          ; LBA start\n";
        output << "    \n";
        output << "    ; Read sectors\n";
        output << "    mov edi, ext2_block_buffer\n";
        output << "    mov ecx, esi\n";
        output << ".read_loop:\n";
        output << "    push ecx\n";
        output << "    push edi\n";
        output << "    push ebx\n";
        output << "    call _read_sector\n";
        output << "    add esp, 8\n";
        output << "    pop ecx\n";
        output << "    \n";
        output << "    add edi, " << SECTOR_SIZE << "\n";
        output << "    inc ebx\n";
        output << "    loop .read_loop\n";
        output << "    \n";
        output << "    ; Copy to user buffer\n";
        output << "    mov ecx, [ext2_block_size]\n";
        output << "    mov esi, ext2_block_buffer\n";
        output << "    mov edi, [ebp + 12]\n";
        output << "    rep movsb\n";
        output << "    \n";
        output << "    ; Update cache (LRU)\n";
        output << "    ; Find least recently used cache slot\n";
        output << "    mov eax, [ext2_cache_lru_counter]\n";
        output << "    inc eax\n";
        output << "    mov [ext2_cache_lru_counter], eax\n";
        output << "    \n";
        output << "    mov ecx, " << EXT2_BLOCK_CACHE_SIZE << "\n";
        output << "    mov esi, ext2_cache_block_numbers\n";
        output << "    xor edx, edx\n";
        output << ".find_slot:\n";
        output << "    cmp dword [esi], 0\n";
        output << "    je .use_slot\n";
        output << "    add esi, 4\n";
        output << "    inc edx\n";
        output << "    loop .find_slot\n";
        output << "    \n";
        output << "    ; All slots used, replace oldest\n";
        output << "    ; For simplicity, use round-robin\n";
        output << "    mov edx, 0\n";
        output << "    \n";
        output << ".use_slot:\n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    mov [ext2_cache_block_numbers + edx*4], eax\n";
        output << "    mov ecx, [ext2_block_size]\n";
        output << "    mov esi, [ebp + 12]\n";
        output << "    mov edi, ext2_cache_data\n";
        output << "    imul edi, edx, 4096\n";
        output << "    add edi, ext2_cache_data\n";
        output << "    rep movsb\n";
        output << "    mov byte [ext2_cache_dirty + edx], 0\n";
        output << "    \n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".cache_hit:\n";
        output << "    ; Copy from cache\n";
        output << "    mov ecx, [ext2_block_size]\n";
        output << "    mov esi, ext2_cache_data\n";
        output << "    imul esi, edx, 4096\n";
        output << "    add esi, ext2_cache_data\n";
        output << "    mov edi, [ebp + 12]\n";
        output << "    rep movsb\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_ext2_write_block:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov ebx, [ebp + 8]    ; block number\n";
        output << "    mov esi, [ebp + 12]   ; buffer pointer\n";
        output << "    \n";
        output << "    ; Update cache\n";
        output << "    mov ecx, " << EXT2_BLOCK_CACHE_SIZE << "\n";
        output << "    mov edi, ext2_cache_block_numbers\n";
        output << "    xor edx, edx\n";
        output << ".find_cache:\n";
        output << "    cmp [edi], ebx\n";
        output << "    je .update_cache\n";
        output << "    add edi, 4\n";
        output << "    inc edx\n";
        output << "    loop .find_cache\n";
        output << "    \n";
        output << "    ; Not in cache, find empty slot\n";
        output << "    mov ecx, " << EXT2_BLOCK_CACHE_SIZE << "\n";
        output << "    mov edi, ext2_cache_block_numbers\n";
        output << "    xor edx, edx\n";
        output << ".find_empty:\n";
        output << "    cmp dword [edi], 0\n";
        output << "    je .add_to_cache\n";
        output << "    add edi, 4\n";
        output << "    inc edx\n";
        output << "    loop .find_empty\n";
        output << "    \n";
        output << "    ; No empty slot, use first slot\n";
        output << "    xor edx, edx\n";
        output << "    \n";
        output << ".add_to_cache:\n";
        output << "    mov [ext2_cache_block_numbers + edx*4], ebx\n";
        output << ".update_cache:\n";
        output << "    mov ecx, [ext2_block_size]\n";
        output << "    mov edi, ext2_cache_data\n";
        output << "    imul edi, edx, 4096\n";
        output << "    add edi, ext2_cache_data\n";
        output << "    rep movsb\n";
        output << "    mov byte [ext2_cache_dirty + edx], 1\n";
        output << "    \n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_ext2_flush_cache:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov ecx, " << EXT2_BLOCK_CACHE_SIZE << "\n";
        output << "    xor edx, edx\n";
        output << ".flush_loop:\n";
        output << "    cmp byte [ext2_cache_dirty + edx], 0\n";
        output << "    je .next\n";
        output << "    \n";
        output << "    ; Get block number\n";
        output << "    mov ebx, [ext2_cache_block_numbers + edx*4]\n";
        output << "    test ebx, ebx\n";
        output << "    jz .next\n";
        output << "    \n";
        output << "    ; Calculate LBA\n";
        output << "    mov eax, [ext2_block_size]\n";
        output << "    mov esi, " << SECTOR_SIZE << "\n";
        output << "    div esi               ; sectors per block\n";
        output << "    mov edi, eax\n";
        output << "    \n";
        output << "    mov eax, ebx\n";
        output << "    mul edi\n";
        output << "    add eax, [ext2_device_start_lba]\n";
        output << "    mov ebx, eax\n";
        output << "    \n";
        output << "    ; Write sectors\n";
        output << "    mov esi, ext2_cache_data\n";
        output << "    imul esi, edx, 4096\n";
        output << "    add esi, ext2_cache_data\n";
        output << "    mov ecx, edi\n";
        output << ".write_loop:\n";
        output << "    push ecx\n";
        output << "    push esi\n";
        output << "    push ebx\n";
        output << "    call _write_sector\n";
        output << "    add esp, 8\n";
        output << "    pop ecx\n";
        output << "    \n";
        output << "    add esi, " << SECTOR_SIZE << "\n";
        output << "    inc ebx\n";
        output << "    loop .write_loop\n";
        output << "    \n";
        output << "    mov byte [ext2_cache_dirty + edx], 0\n";
        output << "    \n";
        output << ".next:\n";
        output << "    inc edx\n";
        output << "    cmp edx, " << EXT2_BLOCK_CACHE_SIZE << "\n";
        output << "    jl .flush_loop\n";
        output << "    \n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; EXT2 mount - initialize and mount EXT2 filesystem\n";
        output << "_ext2_mount:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    ; Read superblock (block 0, offset 1024)\n";
        output << "    ; First, read block 0\n";
        output << "    push ext2_block_buffer\n";
        output << "    push dword [ext2_device_start_lba]\n";
        output << "    call _read_sector\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Read second sector for superblock (if block size > 512)\n";
        output << "    push ext2_block_buffer + 512\n";
        output << "    push dword [ext2_device_start_lba] + 1\n";
        output << "    call _read_sector\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Check superblock magic number at offset 1024+56\n";
        output << "    mov ax, [ext2_block_buffer + 1024 + 56]\n";
        output << "    cmp ax, " << EXT2_MAGIC << "\n";
        output << "    jne .not_ext2\n";
        output << "    \n";
        output << "    ; Read superblock fields\n";
        output << "    mov eax, [ext2_block_buffer + 1024 + 24]  ; inode count\n";
        output << "    mov ebx, [ext2_block_buffer + 1024 + 28]  ; block count\n";
        output << "    mov ecx, [ext2_block_buffer + 1024 + 32]  ; reserved blocks\n";
        output << "    mov edx, [ext2_block_buffer + 1024 + 36]  ; free blocks\n";
        output << "    \n";
        output << "    ; Block size (2^(block_size_shift + 10))\n";
        output << "    movzx eax, word [ext2_block_buffer + 1024 + 24 + 32 + 12]\n";
        output << "    mov ecx, 10\n";
        output << "    add eax, ecx\n";
        output << "    mov ecx, 1\n";
        output << "    shl ecx, al\n";
        output << "    mov [ext2_block_size], ecx\n";
        output << "    \n";
        output << "    ; Inode size\n";
        output << "    movzx eax, word [ext2_block_buffer + 1024 + 24 + 32 + 16]\n";
        output << "    test eax, eax\n";
        output << "    jnz .inode_size_ok\n";
        output << "    mov eax, 128\n";
        output << ".inode_size_ok:\n";
        output << "    mov [ext2_inode_size], eax\n";
        output << "    \n";
        output << "    ; Blocks per group\n";
        output << "    mov eax, [ext2_block_buffer + 1024 + 24 + 32 + 24]\n";
        output << "    mov [ext2_blocks_per_group], eax\n";
        output << "    \n";
        output << "    ; Inodes per group\n";
        output << "    mov eax, [ext2_block_buffer + 1024 + 24 + 32 + 28]\n";
        output << "    mov [ext2_inodes_per_group], eax\n";
        output << "    \n";
        output << "    ; First data block\n";
        output << "    mov eax, [ext2_block_buffer + 1024 + 20]\n";
        output << "    mov [ext2_first_data_block], eax\n";
        output << "    \n";
        output << "    ; Calculate inode table location\n";
        output << "    ; For group 0, inode table starts at block 3 + first_data_block\n";
        output << "    mov eax, [ext2_first_data_block]\n";
        output << "    add eax, 3\n";
        output << "    mov [ext2_inode_table_offset], eax\n";
        output << "    \n";
        output << "    ; Root inode is always 2\n";
        output << "    mov dword [ext2_root_inode], 2\n";
        output << "    \n";
        output << "    ; Mark as mounted\n";
        output << "    mov byte [ext2_mounted], 1\n";
        output << "    \n";
        output << "    ; Copy mount point\n";
        output << "    mov esi, [ebp + 12]\n";
        output << "    mov edi, ext2_mount_point\n";
        output << "    mov ecx, 256\n";
        output << "    rep movsb\n";
        output << "    \n";
        output << "    mov eax, 1\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".not_ext2:\n";
        output << "    xor eax, eax\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; EXT2 read inode\n";
        output << "_ext2_read_inode:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov ebx, [ebp + 8]    ; inode number (1-based)\n";
        output << "    mov edi, [ebp + 12]   ; buffer for inode\n";
        output << "    \n";
        output << "    ; Calculate group and inode index\n";
        output << "    dec ebx               ; 0-based inode\n";
        output << "    mov eax, ebx\n";
        output << "    mov ecx, [ext2_inodes_per_group]\n";
        output << "    xor edx, edx\n";
        output << "    div ecx               ; eax = group, edx = inode index in group\n";
        output << "    \n";
        output << "    ; Calculate block group descriptor location\n";
        output << "    ; Block group descriptors start at block 2 + first_data_block\n";
        output << "    mov ecx, [ext2_first_data_block]\n";
        output << "    add ecx, 2\n";
        output << "    push ecx\n";
        output << "    push ext2_block_buffer\n";
        output << "    call _ext2_read_block\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Get inode table block for this group\n";
        output << "    ; Block group descriptor is 32 bytes\n";
        output << "    imul eax, eax, 32\n";
        output << "    add eax, 8           ; offset to inode table block\n";
        output << "    mov esi, ext2_block_buffer\n";
        output << "    add esi, eax\n";
        output << "    mov ecx, [esi]        ; inode table block\n";
        output << "    \n";
        output << "    ; Read inode table block\n";
        output << "    push ext2_block_buffer\n";
        output << "    push ecx\n";
        output << "    call _ext2_read_block\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Calculate offset within inode table\n";
        output << "    mov eax, [ext2_inode_size]\n";
        output << "    mul edx               ; eax = offset in bytes\n";
        output << "    \n";
        output << "    ; Copy inode to buffer\n";
        output << "    mov esi, ext2_block_buffer\n";
        output << "    add esi, eax\n";
        output << "    mov ecx, [ext2_inode_size]\n";
        output << "    rep movsb\n";
        output << "    \n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; EXT2 read file data using block pointers\n";
        output << "_ext2_read_file:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov ebx, [ebp + 8]    ; inode pointer\n";
        output << "    mov esi, [ebp + 12]   ; file position\n";
        output << "    mov edi, [ebp + 16]   ; buffer\n";
        output << "    mov ecx, [ebp + 20]   ; size to read\n";
        output << "    xor edx, edx          ; bytes read\n";
        output << "    \n";
        output << "    ; Get block size\n";
        output << "    mov eax, [ext2_block_size]\n";
        output << "    \n";
        output << ".read_loop:\n";
        output << "    test ecx, ecx\n";
        output << "    jz .done\n";
        output << "    \n";
        output << "    ; Calculate block number in file\n";
        output << "    mov eax, esi\n";
        output << "    mov ebx, [ext2_block_size]\n";
        output << "    xor edx, edx\n";
        output << "    div ebx               ; eax = block index, edx = offset in block\n";
        output << "    \n";
        output << "    ; Get physical block number from inode\n";
        output << "    cmp eax, 12\n";
        output << "    jl .direct_block\n";
        output << "    sub eax, 12\n";
        output << "    cmp eax, [ext2_block_size] / 4\n";
        output << "    jl .indirect_block\n";
        output << "    ; Double indirect not implemented for simplicity\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".direct_block:\n";
        output << "    ; Direct block pointer at offset 40 + eax*4\n";
        output << "    mov ebx, [ebp + 8]\n";
        output << "    mov ebx, [ebx + 40 + eax*4]\n";
        output << "    test ebx, ebx\n";
        output << "    jz .done\n";
        output << "    jmp .read_block\n";
        output << "    \n";
        output << ".indirect_block:\n";
        output << "    ; Single indirect block\n";
        output << "    push eax\n";
        output << "    mov ebx, [ebp + 8]\n";
        output << "    mov ebx, [ebx + 40 + 12*4]  ; indirect block pointer\n";
        output << "    push ext2_block_buffer\n";
        output << "    push ebx\n";
        output << "    call _ext2_read_block\n";
        output << "    add esp, 8\n";
        output << "    pop eax\n";
        output << "    mov ebx, [ext2_block_size]\n";
        output << "    shr ebx, 2            ; number of pointers per block\n";
        output << "    xor edx, edx\n";
        output << "    div ebx\n";
        output << "    mov eax, [ext2_block_buffer + edx*4]\n";
        output << "    mov ebx, eax\n";
        output << "    \n";
        output << ".read_block:\n";
        output << "    ; Read block\n";
        output << "    push ext2_block_buffer\n";
        output << "    push ebx\n";
        output << "    call _ext2_read_block\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Calculate bytes to copy\n";
        output << "    push ecx\n";
        output << "    mov ecx, [ext2_block_size]\n";
        output << "    sub ecx, edx          ; bytes remaining in block\n";
        output << "    cmp ecx, [ebp + 20]\n";
        output << "    cmovg ecx, [ebp + 20]\n";
        output << "    \n";
        output << "    ; Copy data\n";
        output << "    mov esi, ext2_block_buffer\n";
        output << "    add esi, edx\n";
        output << "    mov edi, [ebp + 16]\n";
        output << "    rep movsb\n";
        output << "    pop ecx\n";
        output << "    \n";
        output << "    ; Update counters\n";
        output << "    sub ecx, [ebp + 20]\n";
        output << "    add esi, [ebp + 20]\n";
        output << "    add edx, [ebp + 20]\n";
        output << "    add [ebp + 16], [ebp + 20]\n";
        output << "    add [ebp + 12], [ebp + 20]\n";
        output << "    \n";
        output << "    jmp .read_loop\n";
        output << "    \n";
        output << ".done:\n";
        output << "    mov eax, edx\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; EXT2 lookup path\n";
        output << "_ext2_lookup:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov esi, [ebp + 8]    ; path string\n";
        output << "    \n";
        output << "    ; Start from root inode\n";
        output << "    mov ebx, [ext2_root_inode]\n";
        output << "    sub esp, " << EXT2_INODE_SIZE << "\n";
        output << "    mov edi, esp\n";
        output << "    push edi\n";
        output << "    push ebx\n";
        output << "    call _ext2_read_inode\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Check if absolute path\n";
        output << "    cmp byte [esi], '/'\n";
        output << "    jne .relative\n";
        output << "    inc esi\n";
        output << "    \n";
        output << ".relative:\n";
        output << "    ; Parse path components\n";
        output << ".parse_loop:\n";
        output << "    cmp byte [esi], 0\n";
        output << "    je .found\n";
        output << "    \n";
        output << "    ; Extract next component\n";
        output << "    mov edi, ext2_block_buffer\n";
        output << "    xor ecx, ecx\n";
        output << ".copy_component:\n";
        output << "    mov al, [esi + ecx]\n";
        output << "    cmp al, '/'\n";
        output << "    je .component_end\n";
        output << "    cmp al, 0\n";
        output << "    je .component_end\n";
        output << "    mov [edi + ecx], al\n";
        output << "    inc ecx\n";
        output << "    jmp .copy_component\n";
        output << ".component_end:\n";
        output << "    mov byte [edi + ecx], 0\n";
        output << "    \n";
        output << "    ; Look up in current directory\n";
        output << "    ; Directory entries are in blocks\n";
        output << "    mov eax, [ext2_block_size]\n";
        output << "    push eax\n";
        output << "    push ext2_block_buffer\n";
        output << "    push edi\n";
        output << "    push dword [ebp + 12]\n";
        output << "    call _ext2_find_entry\n";
        output << "    add esp, 16\n";
        output << "    \n";
        output << "    test eax, eax\n";
        output << "    jz .not_found\n";
        output << "    \n";
        output << "    ; Read next inode\n";
        output << "    mov ebx, eax\n";
        output << "    mov edi, esp\n";
        output << "    push edi\n";
        output << "    push ebx\n";
        output << "    call _ext2_read_inode\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Move to next component\n";
        output << "    add esi, ecx\n";
        output << "    cmp byte [esi], '/'\n";
        output << "    jne .parse_loop\n";
        output << "    inc esi\n";
        output << "    jmp .parse_loop\n";
        output << "    \n";
        output << ".found:\n";
        output << "    ; Return inode number\n";
        output << "    mov eax, [esp + 4]   ; inode number from stack\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".not_found:\n";
        output << "    xor eax, eax\n";
        output << "    \n";
        output << ".done:\n";
        output << "    add esp, " << EXT2_INODE_SIZE << "\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; EXT2 find directory entry\n";
        output << "_ext2_find_entry:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov edi, [ebp + 8]    ; inode pointer\n";
        output << "    mov esi, [ebp + 12]   ; name\n";
        output << "    \n";
        output << "    ; Get file size from inode\n";
        output << "    mov ecx, [edi + 4]    ; size low\n";
        output << "    \n";
        output << "    ; Read directory blocks\n";
        output << "    mov edx, 0            ; offset\n";
        output << "    mov ebx, [ext2_block_size]\n";
        output << "    \n";
        output << ".block_loop:\n";
        output << "    cmp edx, ecx\n";
        output << "    jge .not_found\n";
        output << "    \n";
        output << "    ; Get block number\n";
        output << "    mov eax, edx\n";
        output << "    xor edx, edx\n";
        output << "    div ebx\n";
        output << "    \n";
        output << "    cmp eax, 12\n";
        output << "    jl .direct_block\n";
        output << "    ; Indirect not implemented for simplicity\n";
        output << "    jmp .not_found\n";
        output << "    \n";
        output << ".direct_block:\n";
        output << "    mov eax, [edi + 40 + eax*4]\n";
        output << "    test eax, eax\n";
        output << "    jz .not_found\n";
        output << "    \n";
        output << "    ; Read block\n";
        output << "    push ext2_block_buffer\n";
        output << "    push eax\n";
        output << "    call _ext2_read_block\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Scan directory entries in this block\n";
        output << "    mov eax, 0            ; offset in block\n";
        output << ".entry_loop:\n";
        output << "    cmp eax, ebx\n";
        output << "    jge .next_block\n";
        output << "    \n";
        output << "    mov edx, ext2_block_buffer\n";
        output << "    add edx, eax\n";
        output << "    \n";
        output << "    movzx ecx, byte [edx + 6]  ; name length\n";
        output << "    test ecx, ecx\n";
        output << "    jz .next_entry\n";
        output << "    \n";
        output << "    ; Compare name\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push ecx\n";
        output << "    cld\n";
        output << "    repe cmpsb\n";
        output << "    pop ecx\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    je .found\n";
        output << "    \n";
        output << ".next_entry:\n";
        output << "    movzx ecx, word [edx]      ; entry length\n";
        output << "    add eax, ecx\n";
        output << "    jmp .entry_loop\n";
        output << "    \n";
        output << ".next_block:\n";
        output << "    add edx, ebx\n";
        output << "    jmp .block_loop\n";
        output << "    \n";
        output << ".found:\n";
        output << "    movzx eax, word [edx + 4]  ; inode number\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".not_found:\n";
        output << "    xor eax, eax\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; EXT2 API functions\n";
        output << "_ext2_open:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    \n";
        output << "    ; Check if filesystem mounted\n";
        output << "    cmp byte [ext2_mounted], 1\n";
        output << "    jne .error\n";
        output << "    \n";
        output << "    ; Lookup file\n";
        output << "    push dword [ebp + 8]   ; path\n";
        output << "    call _ext2_lookup\n";
        output << "    add esp, 4\n";
        output << "    \n";
        output << "    test eax, eax\n";
        output << "    jz .error\n";
        output << "    \n";
        output << "    ; Find free file descriptor\n";
        output << "    mov ecx, " << EXT2_MAX_OPEN << "\n";
        output << "    mov edx, ext2_open_files\n";
        output << "    xor ebx, ebx\n";
        output << ".find_free:\n";
        output << "    cmp byte [edx], 0\n";
        output << "    je .found_free\n";
        output << "    inc edx\n";
        output << "    inc ebx\n";
        output << "    loop .find_free\n";
        output << "    jmp .error\n";
        output << "    \n";
        output << ".found_free:\n";
        output << "    mov byte [edx], 1\n";
        output << "    mov [ext2_file_inodes + ebx*4], eax\n";
        output << "    mov dword [ext2_file_positions + ebx*4], 0\n";
        output << "    mov ecx, [ebp + 12]   ; flags\n";
        output << "    mov [ext2_file_mode + ebx*4], ecx\n";
        output << "    mov eax, ebx\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".error:\n";
        output << "    mov eax, -1\n";
        output << ".done:\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_ext2_read:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    \n";
        output << "    mov ebx, [ebp + 8]    ; fd\n";
        output << "    cmp byte [ext2_open_files + ebx], 0\n";
        output << "    je .error\n";
        output << "    \n";
        output << "    ; Read inode\n";
        output << "    mov eax, [ext2_file_inodes + ebx*4]\n";
        output << "    sub esp, " << EXT2_INODE_SIZE << "\n";
        output << "    mov edx, esp\n";
        output << "    push edx\n";
        output << "    push eax\n";
        output << "    call _ext2_read_inode\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Read file data\n";
        output << "    push dword [ebp + 16]  ; size\n";
        output << "    push dword [ebp + 12]  ; buffer\n";
        output << "    push dword [ext2_file_positions + ebx*4]  ; position\n";
        output << "    push edx               ; inode pointer\n";
        output << "    call _ext2_read_file\n";
        output << "    add esp, 16\n";
        output << "    \n";
        output << "    ; Update position\n";
        output << "    add [ext2_file_positions + ebx*4], eax\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".error:\n";
        output << "    xor eax, eax\n";
        output << ".done:\n";
        output << "    add esp, " << EXT2_INODE_SIZE << "\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_ext2_write:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    \n";
        output << "    mov ebx, [ebp + 8]    ; fd\n";
        output << "    cmp byte [ext2_open_files + ebx], 0\n";
        output << "    je .error\n";
        output << "    \n";
        output << "    ; Check write mode\n";
        output << "    mov eax, [ext2_file_mode + ebx*4]\n";
        output << "    test eax, " << (O_WRONLY | O_RDWR) << "\n";
        output << "    jz .error\n";
        output << "    \n";
        output << "    ; For now, just return size (simplified)\n";
        output << "    ; Full write implementation would allocate blocks, etc.\n";
        output << "    mov eax, [ebp + 16]\n";
        output << "    add [ext2_file_positions + ebx*4], eax\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".error:\n";
        output << "    xor eax, eax\n";
        output << ".done:\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_ext2_close:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    cmp eax, " << EXT2_MAX_OPEN << "\n";
        output << "    jge .done\n";
        output << "    cmp byte [ext2_open_files + eax], 0\n";
        output << "    je .done\n";
        output << "    \n";
        output << "    mov byte [ext2_open_files + eax], 0\n";
        output << "    mov dword [ext2_file_inodes + eax*4], 0\n";
        output << "    mov dword [ext2_file_positions + eax*4], 0\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_ext2_mkdir:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    \n";
        output << "    ; Simplified - just return success\n";
        output << "    mov eax, 1\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_ext2_stat:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    \n";
        output << "    ; Lookup file\n";
        output << "    push dword [ebp + 8]   ; path\n";
        output << "    call _ext2_lookup\n";
        output << "    add esp, 4\n";
        output << "    \n";
        output << "    test eax, eax\n";
        output << "    jz .error\n";
        output << "    \n";
        output << "    ; Read inode\n";
        output << "    sub esp, " << EXT2_INODE_SIZE << "\n";
        output << "    mov ebx, esp\n";
        output << "    push ebx\n";
        output << "    push eax\n";
        output << "    call _ext2_read_inode\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Fill stat buffer\n";
        output << "    mov edi, [ebp + 12]   ; statbuf\n";
        output << "    mov eax, [ebx + 4]    ; size\n";
        output << "    mov [edi + 0], eax    ; st_size\n";
        output << "    movzx eax, word [ebx + 2]  ; mode\n";
        output << "    mov [edi + 4], eax    ; st_mode\n";
        output << "    mov eax, [ebx + 8]    ; blocks\n";
        output << "    mov [edi + 8], eax    ; st_blocks\n";
        output << "    \n";
        output << "    mov eax, 1\n";
        output << "    add esp, " << EXT2_INODE_SIZE << "\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".error:\n";
        output << "    xor eax, eax\n";
        output << ".done:\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_ext2_unlink:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    \n";
        output << "    ; Simplified - just return success\n";
        output << "    mov eax, 1\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_ext2_rename:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    \n";
        output << "    ; Simplified - just return success\n";
        output << "    mov eax, 1\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
    }
    
    void generateFileSystem() {
        output << "; ============================================================\n";
        output << "; FAT12 File System Functions (Legacy)\n";
        output << "; ============================================================\n\n";
        
        output << "; Initialize file system (FAT12)\n";
        output << "_fs_init:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    \n";
        output << "    ; Read boot sector\n";
        output << "    push disk_buffer\n";
        output << "    push dword 0\n";
        output << "    call _read_sector\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Read FAT (sectors 1-9)\n";
        output << "    mov ecx, 9\n";
        output << "    mov ebx, 1\n";
        output << "    mov edx, fat12_buffer\n";
        output << ".read_fat:\n";
        output << "    push edx\n";
        output << "    push ebx\n";
        output << "    call _read_sector\n";
        output << "    add esp, 8\n";
        output << "    add edx, 512\n";
        output << "    inc ebx\n";
        output << "    loop .read_fat\n";
        output << "    \n";
        output << "    ; Read root directory (sectors 19-32)\n";
        output << "    mov ecx, 14\n";
        output << "    mov ebx, 19\n";
        output << "    mov edx, root_dir_buffer\n";
        output << ".read_root:\n";
        output << "    push edx\n";
        output << "    push ebx\n";
        output << "    call _read_sector\n";
        output << "    add esp, 8\n";
        output << "    add edx, 512\n";
        output << "    inc ebx\n";
        output << "    loop .read_root\n";
        output << "    \n";
        output << "    ; Initialize open files table\n";
        output << "    mov ecx, " << MAX_OPEN_FILES << "\n";
        output << "    mov edx, open_files\n";
        output << ".init_open:\n";
        output << "    mov byte [edx], 0\n";
        output << "    inc edx\n";
        output << "    loop .init_open\n";
        output << "    \n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Find file in root directory\n";
        output << "; Input: filename in [ebp+8]\n";
        output << "; Output: eax = directory entry index (0-223) or -1 if not found\n";
        output << "_find_file:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov esi, [ebp + 8]   ; filename\n";
        output << "    mov edi, root_dir_buffer\n";
        output << "    mov ecx, 224          ; max directory entries\n";
        output << "    xor ebx, ebx          ; entry index\n";
        output << "    \n";
        output << ".search_loop:\n";
        output << "    cmp byte [edi], 0\n";
        output << "    je .not_found\n";
        output << "    cmp byte [edi], 0xE5\n";
        output << "    je .next_entry\n";
        output << "    \n";
        output << "    ; Compare filename (11 bytes, space padded)\n";
        output << "    push ecx\n";
        output << "    push edi\n";
        output << "    push esi\n";
        output << "    mov ecx, 11\n";
        output << "    cld\n";
        output << ".compare:\n";
        output << "    lodsb\n";
        output << "    cmp al, [edi]\n";
        output << "    jne .compare_fail\n";
        output << "    inc edi\n";
        output << "    loop .compare\n";
        output << "    pop esi\n";
        output << "    pop edi\n";
        output << "    pop ecx\n";
        output << "    mov eax, ebx\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".compare_fail:\n";
        output << "    pop esi\n";
        output << "    pop edi\n";
        output << "    pop ecx\n";
        output << "    \n";
        output << ".next_entry:\n";
        output << "    add edi, 32          ; 32 bytes per entry\n";
        output << "    inc ebx\n";
        output << "    loop .search_loop\n";
        output << "    \n";
        output << ".not_found:\n";
        output << "    mov eax, -1\n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Create new file\n";
        output << "_create_file:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    ; Find free directory entry\n";
        output << "    mov edi, root_dir_buffer\n";
        output << "    mov ecx, 224\n";
        output << "    xor ebx, ebx\n";
        output << ".find_free:\n";
        output << "    cmp byte [edi], 0\n";
        output << "    je .found_free\n";
        output << "    cmp byte [edi], 0xE5\n";
        output << "    je .found_free\n";
        output << "    add edi, 32\n";
        output << "    inc ebx\n";
        output << "    loop .find_free\n";
        output << "    xor eax, eax\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".found_free:\n";
        output << "    ; Copy filename (11 bytes)\n";
        output << "    mov esi, [ebp + 8]\n";
        output << "    mov ecx, 11\n";
        output << "    cld\n";
        output << "    rep movsb\n";
        output << "    \n";
        output << "    ; Set attributes (normal file)\n";
        output << "    mov byte [edi], 0x20\n";
        output << "    inc edi\n";
        output << "    \n";
        output << "    ; Set first cluster\n";
        output << "    mov word [edi], 2\n";
        output << "    add edi, 2\n";
        output << "    \n";
        output << "    ; Set file size\n";
        output << "    mov dword [edi], 0\n";
        output << "    \n";
        output << "    ; Update FAT\n";
        output << "    mov word [fat12_buffer + 2*2], 0xFFF\n";
        output << "    \n";
        output << "    ; Write back FAT and root directory\n";
        output << "    call _write_fat\n";
        output << "    call _write_root_dir\n";
        output << "    \n";
        output << "    mov eax, 1\n";
        output << ".done:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Write FAT back to disk\n";
        output << "_write_fat:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    \n";
        output << "    mov ecx, 9\n";
        output << "    mov ebx, 1\n";
        output << "    mov edx, fat12_buffer\n";
        output << ".write_loop:\n";
        output << "    push edx\n";
        output << "    push ebx\n";
        output << "    call _write_sector\n";
        output << "    add esp, 8\n";
        output << "    add edx, 512\n";
        output << "    inc ebx\n";
        output << "    loop .write_loop\n";
        output << "    \n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Write root directory back to disk\n";
        output << "_write_root_dir:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    \n";
        output << "    mov ecx, 14\n";
        output << "    mov ebx, 19\n";
        output << "    mov edx, root_dir_buffer\n";
        output << ".write_loop:\n";
        output << "    push edx\n";
        output << "    push ebx\n";
        output << "    call _write_sector\n";
        output << "    add esp, 8\n";
        output << "    add edx, 512\n";
        output << "    inc ebx\n";
        output << "    loop .write_loop\n";
        output << "    \n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Get next cluster in chain\n";
        output << "_next_cluster:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    \n";
        output << "    mov ebx, [ebp + 8]\n";
        output << "    mov eax, ebx\n";
        output << "    shr ebx, 1\n";
        output << "    mov bx, [fat12_buffer + ebx]\n";
        output << "    test eax, 1\n";
        output << "    jz .even\n";
        output << "    shr bx, 4\n";
        output << "    jmp .done\n";
        output << ".even:\n";
        output << "    and bx, 0x0FFF\n";
        output << ".done:\n";
        output << "    movzx eax, bx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; Read file data\n";
        output << "_read_file_data:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov ebx, [ebp + 8]    ; fd\n";
        output << "    mov edi, [ebp + 12]   ; buffer\n";
        output << "    mov ecx, [ebp + 16]   ; size\n";
        output << "    xor edx, edx          ; bytes read\n";
        output << "    \n";
        output << "    ; Get file position\n";
        output << "    mov esi, [file_positions + ebx*4]\n";
        output << "    ; Get current cluster\n";
        output << "    mov eax, [file_sectors + ebx*4]\n";
        output << "    \n";
        output << ".read_loop:\n";
        output << "    test ecx, ecx\n";
        output << "    jz .done\n";
        output << "    \n";
        output << "    ; Calculate sector and offset\n";
        output << "    mov ebx, eax\n";
        output << "    add ebx, 33          ; data area starts at sector 33\n";
        output << "    push disk_buffer\n";
        output << "    push ebx\n";
        output << "    call _read_sector\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Copy data\n";
        output << "    mov esi, disk_buffer\n";
        output << "    add esi, [file_positions + ebp+8*4] ; offset in sector\n";
        output << "    mov ebx, 512\n";
        output << "    sub ebx, [file_positions + ebp+8*4]\n";
        output << "    cmp ecx, ebx\n";
        output << "    cmovg ecx, ebx\n";
        output << "    rep movsb\n";
        output << "    add [file_positions + ebp+8*4], ecx\n";
        output << "    add edx, ecx\n";
        output << "    \n";
        output << "    ; Move to next cluster if needed\n";
        output << "    cmp [file_positions + ebp+8*4], 512\n";
        output << "    jl .continue\n";
        output << "    mov [file_positions + ebp+8*4], 0\n";
        output << "    push eax\n";
        output << "    call _next_cluster\n";
        output << "    add esp, 4\n";
        output << "    mov eax, eax\n";
        output << "    cmp eax, 0xFFF\n";
        output << "    je .done\n";
        output << "    \n";
        output << ".continue:\n";
        output << "    mov ebx, [ebp + 8]\n";
        output << "    jmp .read_loop\n";
        output << "    \n";
        output << ".done:\n";
        output << "    mov eax, edx\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
    }
    
    void generateFileFunctions() {
        output << "; ============================================================\n";
        output << "; FAT12 File System API Functions\n";
        output << "; ============================================================\n\n";
        
        output << "; file_open - open or create a file\n";
        output << "_file_open:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    \n";
        output << "    ; Find free file descriptor\n";
        output << "    mov ecx, " << MAX_OPEN_FILES << "\n";
        output << "    mov edx, open_files\n";
        output << "    xor ebx, ebx\n";
        output << ".find_free:\n";
        output << "    cmp byte [edx], 0\n";
        output << "    je .found_free\n";
        output << "    inc edx\n";
        output << "    inc ebx\n";
        output << "    loop .find_free\n";
        output << "    mov eax, -1\n";
        output << "    jmp .error\n";
        output << "    \n";
        output << ".found_free:\n";
        output << "    ; Find file in directory\n";
        output << "    push dword [ebp + 8]\n";
        output << "    call _find_file\n";
        output << "    add esp, 4\n";
        output << "    \n";
        output << "    ; Check mode\n";
        output << "    mov ecx, [ebp + 12]\n";
        output << "    cmp byte [ecx], 'w'\n";
        output << "    je .create_mode\n";
        output << "    cmp byte [ecx], 'r'\n";
        output << "    je .read_mode\n";
        output << "    \n";
        output << ".create_mode:\n";
        output << "    cmp eax, -1\n";
        output << "    jne .file_exists\n";
        output << "    ; Create new file\n";
        output << "    push dword [ebp + 8]\n";
        output << "    call _create_file\n";
        output << "    add esp, 4\n";
        output << "    test eax, eax\n";
        output << "    jz .error\n";
        output << "    ; Find file again\n";
        output << "    push dword [ebp + 8]\n";
        output << "    call _find_file\n";
        output << "    add esp, 4\n";
        output << "    \n";
        output << ".file_exists:\n";
        output << "    ; Get first cluster\n";
        output << "    mov edx, root_dir_buffer\n";
        output << "    imul eax, eax, 32\n";
        output << "    add edx, eax\n";
        output << "    add edx, 26          ; offset to first cluster\n";
        output << "    movzx ecx, word [edx]\n";
        output << "    \n";
        output << ".read_mode:\n";
        output << "    cmp eax, -1\n";
        output << "    je .error\n";
        output << "    \n";
        output << "    ; Set file descriptor\n";
        output << "    mov edx, open_files\n";
        output << "    add edx, ebx\n";
        output << "    mov byte [edx], 1\n";
        output << "    mov [file_sectors + ebx*4], ecx\n";
        output << "    mov dword [file_positions + ebx*4], 0\n";
        output << "    mov eax, ebx\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".error:\n";
        output << "    mov eax, -1\n";
        output << ".done:\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; file_close - close a file\n";
        output << "_file_close:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    cmp eax, " << MAX_OPEN_FILES << "\n";
        output << "    jge .done\n";
        output << "    mov byte [open_files + eax], 0\n";
        output << "    mov dword [file_sectors + eax*4], 0\n";
        output << "    mov dword [file_positions + eax*4], 0\n";
        output << ".done:\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; file_read - read from file\n";
        output << "_file_read:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    \n";
        output << "    mov ebx, [ebp + 8]\n";
        output << "    cmp byte [open_files + ebx], 0\n";
        output << "    je .error\n";
        output << "    \n";
        output << "    push dword [ebp + 16]\n";
        output << "    push dword [ebp + 12]\n";
        output << "    push ebx\n";
        output << "    call _read_file_data\n";
        output << "    add esp, 12\n";
        output << "    jmp .done\n";
        output << "    \n";
        output << ".error:\n";
        output << "    xor eax, eax\n";
        output << ".done:\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; file_write - write to file\n";
        output << "_file_write:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    \n";
        output << "    mov ebx, [ebp + 8]\n";
        output << "    cmp byte [open_files + ebx], 0\n";
        output << "    je .error\n";
        output << "    \n";
        output << "    mov esi, [ebp + 12]   ; source buffer\n";
        output << "    mov ecx, [ebp + 16]   ; size to write\n";
        output << "    xor edx, edx          ; bytes written\n";
        output << "    \n";
        output << "    ; Get current cluster and position\n";
        output << "    mov edi, [file_sectors + ebx*4]\n";
        output << "    mov eax, [file_positions + ebx*4]\n";
        output << "    \n";
        output << ".write_loop:\n";
        output << "    test ecx, ecx\n";
        output << "    jz .done\n";
        output << "    \n";
        output << "    ; Read current sector\n";
        output << "    push disk_buffer\n";
        output << "    push edi\n";
        output << "    add edi, 33\n";
        output << "    call _read_sector\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Calculate offset and copy data\n";
        output << "    mov ebx, eax\n";
        output << "    and ebx, 511\n";
        output << "    add disk_buffer, ebx\n";
        output << "    mov ebx, 512\n";
        output << "    sub ebx, eax\n";
        output << "    cmp ecx, ebx\n";
        output << "    cmovg ecx, ebx\n";
        output << "    mov edi, disk_buffer\n";
        output << "    rep movsb\n";
        output << "    add eax, ecx\n";
        output << "    add edx, ecx\n";
        output << "    \n";
        output << "    ; Write back sector\n";
        output << "    push disk_buffer\n";
        output << "    push [ebp + 8]\n";
        output << "    call _write_sector\n";
        output << "    add esp, 8\n";
        output << "    \n";
        output << "    ; Check if need to move to next cluster\n";
        output << "    cmp eax, 512\n";
        output << "    jl .continue\n";
        output << "    xor eax, eax\n";
        output << "    push edi\n";
        output << "    call _next_cluster\n";
        output << "    add esp, 4\n";
        output << "    mov edi, eax\n";
        output << "    cmp edi, 0xFFF\n";
        output << "    je .done\n";
        output << "    \n";
        output << ".continue:\n";
        output << "    mov ebx, [ebp + 8]\n";
        output << "    jmp .write_loop\n";
        output << "    \n";
        output << ".done:\n";
        output << "    ; Update position\n";
        output << "    mov [file_positions + ebx*4], eax\n";
        output << "    mov [file_sectors + ebx*4], edi\n";
        output << "    mov eax, edx\n";
        output << "    jmp .exit\n";
        output << "    \n";
        output << ".error:\n";
        output << "    xor eax, eax\n";
        output << ".exit:\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; file_seek - seek in file\n";
        output << "_file_seek:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    cmp byte [open_files + eax], 0\n";
        output << "    je .error\n";
        output << "    \n";
        output << "    mov eax, [ebp + 12]\n";
        output << "    mov ebx, [ebp + 8]\n";
        output << "    mov [file_positions + ebx*4], eax\n";
        output << "    jmp .done\n";
        output << ".error:\n";
        output << "    xor eax, eax\n";
        output << ".done:\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; file_tell - get current file position\n";
        output << "_file_tell:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    cmp byte [open_files + eax], 0\n";
        output << "    je .error\n";
        output << "    mov eax, [file_positions + eax*4]\n";
        output << "    jmp .done\n";
        output << ".error:\n";
        output << "    xor eax, eax\n";
        output << ".done:\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; file_delete - delete a file\n";
        output << "_file_delete:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    \n";
        output << "    ; Find file\n";
        output << "    push dword [ebp + 8]\n";
        output << "    call _find_file\n";
        output << "    add esp, 4\n";
        output << "    cmp eax, -1\n";
        output << "    je .done\n";
        output << "    \n";
        output << "    ; Mark directory entry as deleted\n";
        output << "    mov ebx, root_dir_buffer\n";
        output << "    imul eax, eax, 32\n";
        output << "    add ebx, eax\n";
        output << "    mov byte [ebx], 0xE5\n";
        output << "    \n";
        output << "    ; Write back root directory\n";
        output << "    call _write_root_dir\n";
        output << "    \n";
        output << ".done:\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "; file_exists - check if file exists\n";
        output << "_file_exists:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    \n";
        output << "    push dword [ebp + 8]\n";
        output << "    call _find_file\n";
        output << "    add esp, 4\n";
        output << "    cmp eax, -1\n";
        output << "    sete al\n";
        output << "    movzx eax, al\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
    }
    
    void generateKeyboardHandler() {
        output << "; Keyboard interrupt handler (IRQ1)\n";
        output << "keyboard_handler:\n";
        output << "    pushad\n";
        output << "    push ds\n";
        output << "    push es\n";
        output << "    push fs\n";
        output << "    push gs\n";
        output << "    \n";
        output << "    mov ax, 0x10\n";
        output << "    mov ds, ax\n";
        output << "    mov es, ax\n";
        output << "    \n";
        output << "    in al, 0x60\n";
        output << "    \n";
        output << "    mov [key_buffer], al\n";
        output << "    mov byte [key_ready], 1\n";
        output << "    \n";
        output << "    mov al, 0x20\n";
        output << "    out 0x20, al\n";
        output << "    \n";
        output << "    pop gs\n";
        output << "    pop fs\n";
        output << "    pop es\n";
        output << "    pop ds\n";
        output << "    popad\n";
        output << "    iret\n";
        output << "\n";
    }
    
    void generateInterruptStubs() {
        output << "; Interrupt stubs\n";
        for(int i = 0; i < 256; i++) {
            output << "interrupt_stub_" << i << ":\n";
            output << "    pushad\n";
            output << "    push ds\n";
            output << "    push es\n";
            output << "    push fs\n";
            output << "    push gs\n";
            output << "    mov ax, 0x10\n";
            output << "    mov ds, ax\n";
            output << "    mov es, ax\n";
            output << "    push dword " << i << "\n";
            output << "    call _interrupt_handler\n";
            output << "    add esp, 4\n";
            output << "    pop gs\n";
            output << "    pop fs\n";
            output << "    pop es\n";
            output << "    pop ds\n";
            output << "    popad\n";
            output << "    iret\n";
            output << "\n";
        }
    }
    
    void generateIdtFunctions() {
        output << "_idt_set_gate:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    \n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    mov ebx, [ebp + 12]\n";
        output << "    mov ecx, [ebp + 16]\n";
        output << "    \n";
        output << "    mov edx, eax\n";
        output << "    shl edx, 3\n";
        output << "    add edx, idt\n";
        output << "    \n";
        output << "    mov ax, bx\n";
        output << "    mov [edx], ax\n";
        output << "    mov word [edx + 2], 0x08\n";
        output << "    mov ax, cx\n";
        output << "    mov [edx + 4], ax\n";
        output << "    shr ebx, 16\n";
        output << "    mov [edx + 6], bx\n";
        output << "    \n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_idt_init:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    \n";
        output << "    mov ecx, 256\n";
        output << "    mov ebx, idt\n";
        output << ".init_loop:\n";
        output << "    mov dword [ebx], 0\n";
        output << "    mov dword [ebx + 4], 0\n";
        output << "    add ebx, 8\n";
        output << "    loop .init_loop\n";
        output << "    \n";
        output << "    push dword 0x8E\n";
        output << "    push dword interrupt_stub_33\n";
        output << "    push dword 33\n";
        output << "    call _idt_set_gate\n";
        output << "    add esp, 12\n";
        output << "    \n";
        output << "    ; Set timer interrupt (IRQ0 -> interrupt 32)\n";
        output << "    push dword 0x8E\n";
        output << "    push dword timer_handler\n";
        output << "    push dword 32\n";
        output << "    call _idt_set_gate\n";
        output << "    add esp, 12\n";
        output << "    \n";
        output << "    ; Set mouse interrupt (IRQ12 -> interrupt 44)\n";
        output << "    push dword 0x8E\n";
        output << "    push dword mouse_handler\n";
        output << "    push dword 44\n";
        output << "    call _idt_set_gate\n";
        output << "    add esp, 12\n";
        output << "    \n";
        output << "    mov word [idt_ptr], 256*8 - 1\n";
        output << "    mov dword [idt_ptr + 2], idt\n";
        output << "    lidt [idt_ptr]\n";
        output << "    \n";
        output << "    ; Initialize PIT\n";
        output << "    call _init_pit\n";
        output << "    \n";
        output << "    ; Initialize mouse\n";
        output << "    call _mouse_init\n";
        output << "    \n";
        output << "    ; Initialize VESA graphics\n";
        output << "    call _vesa_init\n";
        output << "    \n";
        output << "    ; Remap PIC\n";
        output << "    mov al, ICW1_INIT or ICW1_ICW4\n";
        output << "    out PIC1_COMMAND, al\n";
        output << "    out PIC2_COMMAND, al\n";
        output << "    \n";
        output << "    mov al, 0x20\n";
        output << "    out PIC1_DATA, al\n";
        output << "    mov al, 0x28\n";
        output << "    out PIC2_DATA, al\n";
        output << "    \n";
        output << "    mov al, 0x04\n";
        output << "    out PIC1_DATA, al\n";
        output << "    mov al, 0x02\n";
        output << "    out PIC2_DATA, al\n";
        output << "    \n";
        output << "    mov al, ICW4_8086\n";
        output << "    out PIC1_DATA, al\n";
        output << "    out PIC2_DATA, al\n";
        output << "    \n";
        output << "    ; Unmask timer interrupt (IRQ0) and mouse interrupt (IRQ12)\n";
        output << "    in al, PIC1_DATA\n";
        output << "    and al, 0xFE\n";
        output << "    out PIC1_DATA, al\n";
        output << "    \n";
        output << "    in al, PIC2_DATA\n";
        output << "    and al, 0xEF          ; Unmask IRQ12 (bit 4)\n";
        output << "    out PIC2_DATA, al\n";
        output << "    \n";
        output << "    sti\n";
        output << "    \n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_interrupt_handler:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    cmp eax, 33\n";
        output << "    jne .check_timer\n";
        output << "    call keyboard_handler\n";
        output << "    jmp .done\n";
        output << ".check_timer:\n";
        output << "    cmp eax, 32\n";
        output << "    jne .check_mouse\n";
        output << "    call timer_handler\n";
        output << "    jmp .done\n";
        output << ".check_mouse:\n";
        output << "    cmp eax, 44\n";
        output << "    jne .done\n";
        output << "    call mouse_handler\n";
        output << ".done:\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
    }
    
    void generateKeyboardFunctions() {
        output << "_getchar:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << ".wait_loop:\n";
        output << "    cmp byte [key_ready], 1\n";
        output << "    jne .wait_loop\n";
        output << "    mov al, [key_buffer]\n";
        output << "    mov byte [key_ready], 0\n";
        output << "    movzx eax, al\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "_is_key_pressed:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    xor eax, eax\n";
        output << "    cmp byte [key_ready], 1\n";
        output << "    jne .done\n";
        output << "    mov al, 1\n";
        output << ".done:\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
    }
    
    void generatePrintFunction() {
        output << "print_string:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    \n";
        output << "    mov esi, [ebp + 8]\n";
        output << ".print_loop:\n";
        output << "    mov al, [esi]\n";
        output << "    test al, al\n";
        output << "    jz .print_done\n";
        output << "    cmp al, 10\n";
        output << "    je .handle_newline\n";
        output << "    cmp al, 9\n";
        output << "    je .handle_tab\n";
        output << "    call put_char\n";
        output << "    inc esi\n";
        output << "    jmp .print_loop\n";
        output << ".handle_newline:\n";
        output << "    call newline\n";
        output << "    inc esi\n";
        output << "    jmp .print_loop\n";
        output << ".handle_tab:\n";
        output << "    call tab\n";
        output << "    inc esi\n";
        output << "    jmp .print_loop\n";
        output << ".print_done:\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
        
        output << "put_char:\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    movzx ebx, byte [cursor_y]\n";
        output << "    movzx ecx, byte [cursor_x]\n";
        output << "    cmp ecx, " << VGA_WIDTH << "\n";
        output << "    jl .write_char\n";
        output << "    call newline\n";
        output << "    movzx ebx, byte [cursor_y]\n";
        output << "    movzx ecx, byte [cursor_x]\n";
        output << ".write_char:\n";
        output << "    mov eax, " << VGA_WIDTH << "\n";
        output << "    mul ebx\n";
        output << "    add eax, ecx\n";
        output << "    shl eax, 1\n";
        output << "    mov edx, vga_buffer\n";
        output << "    add edx, eax\n";
        output << "    mov [edx], al\n";
        output << "    mov byte [edx + 1], 0x0F\n";
        output << "    inc ecx\n";
        output << "    mov [cursor_x], cl\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    ret\n";
        output << "\n";
        
        output << "newline:\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    movzx ebx, byte [cursor_y]\n";
        output << "    movzx ecx, byte [cursor_x]\n";
        output << "    mov byte [cursor_x], 0\n";
        output << "    inc ebx\n";
        output << "    cmp ebx, " << VGA_HEIGHT << "\n";
        output << "    jl .update_y\n";
        output << "    call scroll_screen\n";
        output << "    mov ebx, " << (VGA_HEIGHT - 1) << "\n";
        output << ".update_y:\n";
        output << "    mov [cursor_y], bl\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    ret\n";
        output << "\n";
        
        output << "tab:\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    movzx ecx, byte [cursor_x]\n";
        output << "    mov eax, ecx\n";
        output << "    add eax, 4\n";
        output << "    and eax, 0xFFFFFFFC\n";
        output << "    sub eax, ecx\n";
        output << "    mov ebx, eax\n";
        output << ".tab_loop:\n";
        output << "    cmp ebx, 0\n";
        output << "    je .tab_done\n";
        output << "    mov al, ' '\n";
        output << "    call put_char\n";
        output << "    dec ebx\n";
        output << "    jmp .tab_loop\n";
        output << ".tab_done:\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    ret\n";
        output << "\n";
        
        output << "scroll_screen:\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    push esi\n";
        output << "    push edi\n";
        output << "    mov esi, vga_buffer + " << (VGA_WIDTH * 2) << "\n";
        output << "    mov edi, vga_buffer\n";
        output << "    mov ecx, " << ((VGA_HEIGHT - 1) * VGA_WIDTH) << "\n";
        output << "    rep movsd\n";
        output << "    mov edi, vga_buffer + " << ((VGA_HEIGHT - 1) * VGA_WIDTH * 2) << "\n";
        output << "    mov ecx, " << VGA_WIDTH << "\n";
        output << "    mov eax, 0x0F20\n";
        output << "    rep stosd\n";
        output << "    pop edi\n";
        output << "    pop esi\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    ret\n";
        output << "\n";
        
        output << "print_number:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    push eax\n";
        output << "    push ebx\n";
        output << "    push ecx\n";
        output << "    push edx\n";
        output << "    mov eax, [ebp + 8]\n";
        output << "    mov ecx, 10\n";
        output << "    xor ebx, ebx\n";
        output << ".push_digits:\n";
        output << "    xor edx, edx\n";
        output << "    div ecx\n";
        output << "    push dx\n";
        output << "    inc ebx\n";
        output << "    test eax, eax\n";
        output << "    jnz .push_digits\n";
        output << ".print_digits:\n";
        output << "    pop dx\n";
        output << "    add dl, '0'\n";
        output << "    mov al, dl\n";
        output << "    call put_char\n";
        output << "    dec ebx\n";
        output << "    jnz .print_digits\n";
        output << "    pop edx\n";
        output << "    pop ecx\n";
        output << "    pop ebx\n";
        output << "    pop eax\n";
        output << "    pop ebp\n";
        output << "    ret\n";
        output << "\n";
    }
    
    void generateExpression(ASTNode* node) {
        if(!node) return;
        
        switch(node->type) {
            case ASTNode::NODE_NUMBER:
                output << "    push dword " << node->value << "\n";
                break;
                
            case ASTNode::NODE_IDENTIFIER:
                if(symbols.find(node->value) != symbols.end()) {
                    SymbolInfo& sym = symbols[node->value];
                    if(sym.isArray) {
                        output << "    lea eax, [ebp - " << sym.offset << "]\n";
                        output << "    push eax\n";
                    } else if(sym.isPointer) {
                        output << "    push dword [ebp - " << sym.offset << "]\n";
                    } else {
                        output << "    push dword [ebp - " << sym.offset << "]\n";
                    }
                } else {
                    output << "    push dword 0\n";
                }
                break;
                
            case ASTNode::NODE_CALL:
                {
                    if(node->value == "getchar") {
                        output << "    call _getchar\n";
                        output << "    push eax\n";
                    } else if(node->value == "is_key_pressed") {
                        output << "    call _is_key_pressed\n";
                        output << "    push eax\n";
                    } else if(node->value == "idt_init") {
                        output << "    call _idt_init\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "idt_set_gate") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _idt_set_gate\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "malloc") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _malloc\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "free") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _free\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "calloc") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _calloc\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "realloc") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _realloc\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "heap_used") {
                        output << "    call _heap_used\n";
                        output << "    push eax\n";
                    } else if(node->value == "heap_free") {
                        output << "    call _heap_free\n";
                        output << "    push eax\n";
                    } else if(node->value == "heap_dump") {
                        output << "    call _heap_dump\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "file_open") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _file_open\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "file_close") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _file_close\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "file_read") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _file_read\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "file_write") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _file_write\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "file_seek") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _file_seek\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "file_tell") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _file_tell\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "file_delete") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _file_delete\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "file_exists") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _file_exists\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "ext2_mount") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _ext2_mount\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "ext2_open") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _ext2_open\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "ext2_read") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _ext2_read\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "ext2_write") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _ext2_write\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "ext2_close") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _ext2_close\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "ext2_mkdir") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _ext2_mkdir\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "ext2_stat") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _ext2_stat\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "ext2_unlink") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _ext2_unlink\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "ext2_rename") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _ext2_rename\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "create_process") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _create_process\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "create_thread") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _create_thread\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "yield") {
                        output << "    call _yield\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "sleep") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _sleep\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "wait_pid") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _wait_pid\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "kill_pid") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _kill_pid\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "getpid") {
                        output << "    call _getpid\n";
                        output << "    push eax\n";
                    } else if(node->value == "getppid") {
                        output << "    call _getppid\n";
                        output << "    push eax\n";
                    } else if(node->value == "set_priority") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _set_priority\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "mouse_init") {
                        output << "    call _mouse_init\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "mouse_get_x") {
                        output << "    call _mouse_get_x\n";
                        output << "    push eax\n";
                    } else if(node->value == "mouse_get_y") {
                        output << "    call _mouse_get_y\n";
                        output << "    push eax\n";
                    } else if(node->value == "mouse_get_buttons") {
                        output << "    call _mouse_get_buttons\n";
                        output << "    push eax\n";
                    } else if(node->value == "mouse_get_dx") {
                        output << "    call _mouse_get_dx\n";
                        output << "    push eax\n";
                    } else if(node->value == "mouse_get_dy") {
                        output << "    call _mouse_get_dy\n";
                        output << "    push eax\n";
                    } else if(node->value == "mouse_set_position") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _mouse_set_position\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "mouse_set_cursor") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _mouse_set_cursor\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "vesa_init") {
                        output << "    call _vesa_init\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "vesa_set_mode") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _vesa_set_mode\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push eax\n";
                    } else if(node->value == "vesa_draw_pixel") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _vesa_draw_pixel\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "vesa_draw_rect") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _vesa_draw_rect\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "vesa_draw_line") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _vesa_draw_line\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "vesa_draw_char") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _vesa_draw_char\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "vesa_draw_string") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _vesa_draw_string\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "vesa_clear") {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _vesa_clear\n";
                        output << "    add esp, " << (node->children.size() * 4) << "\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "vesa_swap_buffers") {
                        output << "    call _vesa_swap_buffers\n";
                        output << "    push dword 0\n";
                    } else if(node->value == "vesa_get_width") {
                        output << "    call _vesa_get_width\n";
                        output << "    push eax\n";
                    } else if(node->value == "vesa_get_height") {
                        output << "    call _vesa_get_height\n";
                        output << "    push eax\n";
                    } else if(node->value == "vesa_get_bpp") {
                        output << "    call _vesa_get_bpp\n";
                        output << "    push eax\n";
                    } else {
                        for(auto child : node->children) {
                            generateExpression(child);
                        }
                        output << "    call _" << node->value << "\n";
                        if(!node->children.empty()) {
                            output << "    add esp, " << (node->children.size() * 4) << "\n";
                        }
                        output << "    push eax\n";
                    }
                }
                break;
                
            case ASTNode::NODE_STRING:
                {
                    int idx = -1;
                    for(size_t i = 0; i < stringLiterals.size(); i++) {
                        if(stringLiterals[i] == node->value) {
                            idx = i;
                            break;
                        }
                    }
                    if(idx == -1) {
                        idx = stringLiterals.size();
                        stringLiterals.push_back(node->value);
                    }
                    output << "    push str" << idx << "\n";
                }
                break;
                
            case ASTNode::NODE_GETCHAR:
                output << "    call _getchar\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_IS_KEY_PRESSED:
                output << "    call _is_key_pressed\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_IDT_INIT:
                output << "    call _idt_init\n";
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_IDT_SET_GATE:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _idt_set_gate\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_MALLOC:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _malloc\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_FREE:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _free\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_CALLOC:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _calloc\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_REALLOC:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _realloc\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_HEAP_USED:
                output << "    call _heap_used\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_HEAP_FREE:
                output << "    call _heap_free\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_HEAP_DUMP:
                output << "    call _heap_dump\n";
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_FILE_OPEN:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _file_open\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_FILE_CLOSE:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _file_close\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_FILE_READ:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _file_read\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_FILE_WRITE:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _file_write\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_FILE_SEEK:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _file_seek\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_FILE_TELL:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _file_tell\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_FILE_DELETE:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _file_delete\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_FILE_EXISTS:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _file_exists\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_EXT2_MOUNT:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _ext2_mount\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_EXT2_OPEN:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _ext2_open\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_EXT2_READ:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _ext2_read\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_EXT2_WRITE:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _ext2_write\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_EXT2_CLOSE:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _ext2_close\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_EXT2_MKDIR:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _ext2_mkdir\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_EXT2_STAT:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _ext2_stat\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_EXT2_UNLINK:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _ext2_unlink\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_EXT2_RENAME:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _ext2_rename\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_CREATE_PROCESS:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _create_process\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_CREATE_THREAD:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _create_thread\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_YIELD:
                output << "    call _yield\n";
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_SLEEP:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _sleep\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_WAIT_PID:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _wait_pid\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_KILL_PID:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _kill_pid\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_GETPID:
                output << "    call _getpid\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_GETPPID:
                output << "    call _getppid\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_SET_PRIORITY:
                for(auto child : node->children) {
                    generateExpression(child);
                }
                output << "    call _set_priority\n";
                output << "    add esp, " << (node->children.size() * 4) << "\n";
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_PORT_IN_B:
                generateExpression(node->children[0]);
                output << "    pop dx\n";
                output << "    xor eax, eax\n";
                output << "    in al, dx\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_PORT_IN_W:
                generateExpression(node->children[0]);
                output << "    pop dx\n";
                output << "    xor eax, eax\n";
                output << "    in ax, dx\n";
                output << "    push eax\n";
                break;
                
            case ASTNode::NODE_PORT_OUT_B:
                generateExpression(node->children[0]);
                generateExpression(node->children[1]);
                output << "    pop eax\n";
                output << "    pop dx\n";
                output << "    out dx, al\n";
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_PORT_OUT_W:
                generateExpression(node->children[0]);
                generateExpression(node->children[1]);
                output << "    pop eax\n";
                output << "    pop dx\n";
                output << "    out dx, ax\n";
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_ADDRESS_OF:
                {
                    ASTNode* operand = node->children[0];
                    if(operand->type == ASTNode::NODE_IDENTIFIER) {
                        SymbolInfo& sym = symbols[operand->value];
                        output << "    lea eax, [ebp - " << sym.offset << "]\n";
                        output << "    push eax\n";
                    } else {
                        generateExpression(operand);
                    }
                }
                break;
                
            case ASTNode::NODE_POINTER_DEREF:
                generateExpression(node->children[0]);
                output << "    pop eax\n";
                output << "    push dword [eax]\n";
                break;
                
            case ASTNode::NODE_NULL_PTR:
                output << "    push dword 0\n";
                break;
                
            case ASTNode::NODE_ARRAY_ACCESS:
                generateExpression(node->children[0]);
                generateExpression(node->children[1]);
                output << "    pop ebx\n";
                output << "    pop eax\n";
                output << "    shl ebx, 2\n";
                output << "    add eax, ebx\n";
                output << "    push dword [eax]\n";
                break;
                
            case ASTNode::NODE_ARRAY_LENGTH:
                {
                    std::string arrayName = node->children[0]->value;
                    if(symbols.find(arrayName) != symbols.end()) {
                        SymbolInfo& sym = symbols[arrayName];
                        int totalSize = 1;
                        for(int dim : sym.dimensions) totalSize *= dim;
                        output << "    push dword " << totalSize << "\n";
                    } else {
                        output << "    push dword 0\n";
                    }
                }
                break;
                
            case ASTNode::NODE_ARRAY_COPY:
                generateExpression(node->children[0]);
                generateExpression(node->children[1]);
                
                {
                    std::string srcName = node->children[0]->value;
                    if(symbols.find(srcName) != symbols.end()) {
                        SymbolInfo& srcSym = symbols[srcName];
                        int totalSize = 1;
                        for(int dim : srcSym.dimensions) totalSize *= dim;
                        totalSize *= getTypeSize("int");
                        
                        output << "    pop edi\n";
                        output << "    pop esi\n";
                        output << "    mov ecx, " << totalSize << "\n";
                        output << "    rep movsb\n";
                        output << "    push dword 1\n";
                    } else {
                        output << "    push dword 0\n";
                    }
                }
                break;
                
            case ASTNode::NODE_BINARY_OP:
                generateExpression(node->children[0]);
                generateExpression(node->children[1]);
                output << "    pop ebx\n";
                output << "    pop eax\n";
                
                if(node->value == "+") output << "    add eax, ebx\n";
                else if(node->value == "-") output << "    sub eax, ebx\n";
                else if(node->value == "*") output << "    imul eax, ebx\n";
                else if(node->value == "/") {
                    output << "    cdq\n";
                    output << "    idiv ebx\n";
                }
                else if(node->value == "%") {
                    output << "    cdq\n";
                    output << "    idiv ebx\n";
                    output << "    mov eax, edx\n";
                }
                else if(node->value == "==") {
                    output << "    cmp eax, ebx\n";
                    output << "    sete al\n";
                    output << "    movzx eax, al\n";
                }
                else if(node->value == "!=") {
                    output << "    cmp eax, ebx\n";
                    output << "    setne al\n";
                    output << "    movzx eax, al\n";
                }
                else if(node->value == "<") {
                    output << "    cmp eax, ebx\n";
                    output << "    setl al\n";
                    output << "    movzx eax, al\n";
                }
                else if(node->value == ">") {
                    output << "    cmp eax, ebx\n";
                    output << "    setg al\n";
                    output << "    movzx eax, al\n";
                }
                else if(node->value == "<=") {
                    output << "    cmp eax, ebx\n";
                    output << "    setle al\n";
                    output << "    movzx eax, al\n";
                }
                else if(node->value == ">=") {
                    output << "    cmp eax, ebx\n";
                    output << "    setge al\n";
                    output << "    movzx eax, al\n";
                }
                output << "    push eax\n";
                break;
                
            default:
                break;
        }
    }
    
    void generateStatement(ASTNode* node) {
        if(!node) return;
        
        switch(node->type) {
            case ASTNode::NODE_VAR_DECL:
                {
                    std::string varType = node->children[0]->value;
                    int typeSize = getTypeSize(varType);
                    stackOffset += typeSize;
                    SymbolInfo sym;
                    sym.type = varType;
                    sym.isArray = false;
                    sym.isPointer = false;
                    sym.offset = stackOffset;
                    symbols[node->value] = sym;
                    
                    if(node->children.size() > 1) {
                        generateExpression(node->children[1]);
                        output << "    pop dword [ebp - " << stackOffset << "]\n";
                    }
                }
                break;
                
            case ASTNode::NODE_POINTER_DECL:
                {
                    int typeSize = 4;
                    stackOffset += typeSize;
                    SymbolInfo sym;
                    sym.type = "ptr";
                    sym.pointedType = node->pointedType;
                    sym.isArray = false;
                    sym.isPointer = true;
                    sym.offset = stackOffset;
                    symbols[node->value] = sym;
                    
                    if(node->children.size() > 1) {
                        generateExpression(node->children[1]);
                        output << "    pop dword [ebp - " << stackOffset << "]\n";
                    }
                }
                break;
                
            case ASTNode::NODE_ARRAY_DECL:
                {
                    std::string varType = node->children[0]->value;
                    int elementSize = getTypeSize(varType);
                    int totalSize = calculateArraySize(node->dimensions, elementSize);
                    totalSize = (totalSize + 3) & ~3;
                    
                    stackOffset += totalSize;
                    SymbolInfo sym;
                    sym.type = varType;
                    sym.isArray = true;
                    sym.isPointer = false;
                    sym.dimensions = node->dimensions;
                    sym.offset = stackOffset;
                    sym.size = totalSize;
                    symbols[node->value] = sym;
                    
                    if(node->children.size() > 1) {
                        if(node->children.size() == 2) {
                            generateExpression(node->children[1]);
                            output << "    pop eax\n";
                            output << "    mov ecx, " << (totalSize / 4) << "\n";
                            output << "    mov edi, ebp\n";
                            output << "    sub edi, " << stackOffset << "\n";
                            output << "    rep stosd\n";
                        } else {
                            int totalElements = 1;
                            for(int dim : node->dimensions) totalElements *= dim;
                            int initCount = node->children.size() - 1;
                            
                            for(int i = 0; i < initCount && i < totalElements; i++) {
                                generateExpression(node->children[i + 1]);
                                output << "    pop eax\n";
                                output << "    mov [ebp - " << (stackOffset - i * 4) << "], eax\n";
                            }
                        }
                    }
                }
                break;
                
            case ASTNode::NODE_ASSIGNMENT:
                {
                    ASTNode* left = node->children[0];
                    generateExpression(node->children[1]);
                    
                    if(left->type == ASTNode::NODE_IDENTIFIER) {
                        SymbolInfo& sym = symbols[left->value];
                        output << "    pop eax\n";
                        output << "    mov [ebp - " << sym.offset << "], eax\n";
                    } else if(left->type == ASTNode::NODE_POINTER_DEREF) {
                        generateExpression(left->children[0]);
                        output << "    pop ebx\n";
                        output << "    pop eax\n";
                        output << "    mov [ebx], eax\n";
                    } else if(left->type == ASTNode::NODE_ARRAY_ACCESS) {
                        generateExpression(left->children[0]);
                        generateExpression(left->children[1]);
                        output << "    pop ebx\n";
                        output << "    pop edi\n";
                        output << "    pop eax\n";
                        output << "    shl ebx, 2\n";
                        output << "    add edi, ebx\n";
                        output << "    mov [edi], eax\n";
                    }
                }
                break;
                
            case ASTNode::NODE_PRINT:
                {
                    generateExpression(node->children[0]);
                    output << "    pop eax\n";
                    
                    if(node->children[0]->type == ASTNode::NODE_STRING) {
                        output << "    push eax\n";
                        output << "    call print_string\n";
                        output << "    add esp, 4\n";
                    } else {
                        output << "    push eax\n";
                        output << "    call print_number\n";
                        output << "    add esp, 4\n";
                    }
                }
                break;
                
            case ASTNode::NODE_RETURN:
                if(!node->children.empty()) {
                    generateExpression(node->children[0]);
                    output << "    pop eax\n";
                }
                output << "    mov esp, ebp\n";
                output << "    pop ebp\n";
                output << "    ret\n";
                break;
                
            case ASTNode::NODE_IF:
                {
                    generateExpression(node->children[0]);
                    std::string elseLabel = newLabel();
                    std::string endLabel = newLabel();
                    
                    output << "    pop eax\n";
                    output << "    test eax, eax\n";
                    output << "    jz " << elseLabel << "\n";
                    
                    generateStatement(node->children[1]);
                    
                    if(node->children.size() > 2) {
                        output << "    jmp " << endLabel << "\n";
                        output << elseLabel << ":\n";
                        generateStatement(node->children[2]);
                        output << endLabel << ":\n";
                    } else {
                        output << elseLabel << ":\n";
                    }
                }
                break;
                
            case ASTNode::NODE_WHILE:
                {
                    std::string startLabel = newLabel();
                    std::string endLabel = newLabel();
                    
                    output << startLabel << ":\n";
                    generateExpression(node->children[0]);
                    output << "    pop eax\n";
                    output << "    test eax, eax\n";
                    output << "    jz " << endLabel << "\n";
                    
                    generateStatement(node->children[1]);
                    
                    output << "    jmp " << startLabel << "\n";
                    output << endLabel << ":\n";
                }
                break;
                
            case ASTNode::NODE_FOR:
                {
                    std::string startLabel = newLabel();
                    std::string endLabel = newLabel();
                    
                    if(node->children.size() >= 2) {
                        generateStatement(node->children[0]);
                    }
                    
                    output << startLabel << ":\n";
                    
                    if(node->children.size() >= 3) {
                        generateExpression(node->children[2]);
                        output << "    pop eax\n";
                        output << "    test eax, eax\n";
                        output << "    jz " << endLabel << "\n";
                    }
                    
                    if(node->children.size() >= 5) {
                        generateStatement(node->children[4]);
                    } else if(node->children.size() >= 4) {
                        generateStatement(node->children[3]);
                    }
                    
                    output << "    jmp " << startLabel << "\n";
                    output << endLabel << ":\n";
                }
                break;
                
            case ASTNode::NODE_BLOCK:
                for(auto child : node->children) {
                    generateStatement(child);
                }
                break;
                
            case ASTNode::NODE_FUNC_DECL:
                {
                    currentFunction = node->value;
                    output << "_" << node->value << ":\n";
                    output << "    push ebp\n";
                    output << "    mov ebp, esp\n";
                    
                    int oldOffset = stackOffset;
                    stackOffset = 0;
                    symbols.clear();
                    
                    for(auto child : node->children) {
                        generateStatement(child);
                    }
                    
                    output << "    mov esp, ebp\n";
                    output << "    pop ebp\n";
                    output << "    ret\n";
                    output << "\n";
                    
                    stackOffset = oldOffset;
                }
                break;
                
            case ASTNode::NODE_ASM:
                if(!node->children.empty()) {
                    output << node->children[0]->value << "\n";
                }
                break;
                
            default:
                break;
        }
    }
    
public:
    CodeGenerator() : stackOffset(0), labelCounter(0) {}
    
    std::string generate(ASTNode* root) {
        output.str("");
        stringLiterals.clear();
        symbols.clear();
        stackOffset = 0;
        labelCounter = 0;
        
        generateMultibootHeader();
        generateEntryPoint();
        generateDataSection();
        generateBssSection();
        
        output << "section .text\n";
        output << "global _kernel_main\n";
        output << "global print_string\n";
        output << "global print_number\n";
        output << "global _getchar\n";
        output << "global _is_key_pressed\n";
        output << "global _idt_init\n";
        output << "global _idt_set_gate\n";
        output << "global _malloc\n";
        output << "global _free\n";
        output << "global _calloc\n";
        output << "global _realloc\n";
        output << "global _heap_used\n";
        output << "global _heap_free\n";
        output << "global _heap_dump\n";
        output << "global _heap_init\n";
        output << "global _read_sector\n";
        output << "global _write_sector\n";
        output << "global _file_open\n";
        output << "global _file_close\n";
        output << "global _file_read\n";
        output << "global _file_write\n";
        output << "global _file_seek\n";
        output << "global _file_tell\n";
        output << "global _file_delete\n";
        output << "global _file_exists\n";
        output << "global _fs_init\n";
        output << "global _ext2_mount\n";
        output << "global _ext2_open\n";
        output << "global _ext2_read\n";
        output << "global _ext2_write\n";
        output << "global _ext2_close\n";
        output << "global _ext2_mkdir\n";
        output << "global _ext2_stat\n";
        output << "global _ext2_unlink\n";
        output << "global _ext2_rename\n";
        output << "global _create_process\n";
        output << "global _create_thread\n";
        output << "global _yield\n";
        output << "global _sleep\n";
        output << "global _wait_pid\n";
        output << "global _kill_pid\n";
        output << "global _getpid\n";
        output << "global _getppid\n";
        output << "global _set_priority\n";
        output << "global _init_pit\n";
        output << "global timer_handler\n";
        output << "global _mouse_init\n";
        output << "global _mouse_get_x\n";
        output << "global _mouse_get_y\n";
        output << "global _mouse_get_buttons\n";
        output << "global _mouse_get_dx\n";
        output << "global _mouse_get_dy\n";
        output << "global _mouse_set_position\n";
        output << "global _mouse_set_cursor\n";
        output << "global mouse_handler\n";
        output << "global _vesa_init\n";
        output << "global _vesa_set_mode\n";
        output << "global _vesa_draw_pixel\n";
        output << "global _vesa_draw_rect\n";
        output << "global _vesa_draw_line\n";
        output << "global _vesa_draw_char\n";
        output << "global _vesa_draw_string\n";
        output << "global _vesa_clear\n";
        output << "global _vesa_swap_buffers\n";
        output << "global _vesa_get_width\n";
        output << "global _vesa_get_height\n";
        output << "global _vesa_get_bpp\n";
        output << "extern keyboard_handler\n";
        output << "extern _interrupt_handler\n";
        output << "\n";
        
        generateScheduler();
        generateHeapManagement();
        generateDiskIO();
        generateEXT2FileSystem();
        generateFileSystem();
        generateFileFunctions();
        generateMouseFunctions();
        generateVesaFunctions();
        generateInterruptStubs();
        generateIdtFunctions();
        generateKeyboardHandler();
        generateKeyboardFunctions();
        
        output << "_kernel_main:\n";
        output << "    push ebp\n";
        output << "    mov ebp, esp\n";
        output << "    \n";
        output << "    ; Initialize heap\n";
        output << "    call _heap_init\n";
        output << "    \n";
        output << "    ; Initialize file system (FAT12)\n";
        output << "    call _fs_init\n";
        output << "    \n";
        output << "    ; Set device LBA start (assuming disk at LBA 0)\n";
        output << "    mov dword [ext2_device_start_lba], 0\n";
        output << "    \n";
        output << "    ; Try to mount EXT2 filesystem\n";
        output << "    push str_ext2_mount_point\n";
        output << "    push str_ext2_device\n";
        output << "    call _ext2_mount\n";
        output << "    add esp, 8\n";
        output << "    test eax, eax\n";
        output << "    jz .no_ext2\n";
        output << "    push str_ext2_mounted\n";
        output << "    call print_string\n";
        output << "    add esp, 4\n";
        output << ".no_ext2:\n";
        output << "    \n";
        output << "    ; Initialize VESA graphics\n";
        output << "    call _vesa_init\n";
        output << "    test eax, eax\n";
        output << "    jz .no_vesa\n";
        output << "    push str_vesa_ok\n";
        output << "    call print_string\n";
        output << "    add esp, 4\n";
        output << ".no_vesa:\n";
        output << "    \n";
        output << "    ; Initialize IDT and PIT\n";
        output << "    call _idt_init\n";
        output << "    \n";
        output << "    ; Create init process (PID 1)\n";
        output << "    push dword 128        ; priority\n";
        output << "    push str_init_name\n";
        output << "    push dword _init_process\n";
        output << "    call _create_process\n";
        output << "    add esp, 12\n";
        output << "    \n";
        output << "    ; Set current process\n";
        output << "    mov [current_pid], eax\n";
        output << "    \n";
        output << "    ; Get PCB and restore state\n";
        output << "    push eax\n";
        output << "    call _get_pcb\n";
        output << "    add esp, 4\n";
        output << "    \n";
        output << "    ; Jump to init process\n";
        output << "    jmp _switch_to\n";
        output << "    \n";
        output << "    ; Should never reach here\n";
        output << "    cli\n";
        output << "    hlt\n";
        output << "\n";
        
        output << "_init_process:\n";
        output << "    ; Init process - runs user code\n";
        output << "    mov eax, [current_pid]\n";
        output << "    push eax\n";
        output << "    call print_string\n";
        output << "    add esp, 4\n";
        output << "    \n";
        
        stackOffset = 0;
        
        for(auto child : root->children) {
            generateStatement(child);
        }
        
        output << "    ; Init process exit\n";
        output << "    mov eax, [current_pid]\n";
        output << "    push eax\n";
        output << "    call _kill_pid\n";
        output << "    add esp, 4\n";
        output << "    \n";
        output << "    ; Schedule next process\n";
        output << "    call _schedule\n";
        output << "    test eax, eax\n";
        output << "    jnz .switch\n";
        output << "    cli\n";
        output << "    hlt\n";
        output << ".switch:\n";
        output << "    jmp _switch_to\n";
        output << "\n";
        
        generatePrintFunction();
        
        output << "section .rodata\n";
        output << "str_init_name db `init`, 0\n";
        output << "str_ext2_device db `/dev/hda`, 0\n";
        output << "str_ext2_mount_point db `/mnt`, 0\n";
        output << "str_ext2_mounted db `EXT2 filesystem mounted\\n`, 0\n";
        output << "str_vesa_ok db `VESA graphics initialized\\n`, 0\n";
        generateStringTable();
        
        return output.str();
    }
    
    void generateStringTable() {
        for(size_t i = 0; i < stringLiterals.size(); i++) {
            std::string escaped;
            for(char c : stringLiterals[i]) {
                if(c == '\n') escaped += "`, 10, `";
                else if(c == '\t') escaped += "`, 9, `";
                else if(c == '\\') escaped += "`, 92, `";
                else if(c == '"') escaped += "`, 34, `";
                else escaped += c;
            }
            output << "    str" << i << " db `" << escaped << "`, 0\n";
        }
        output << "\n";
    }
};

// ============================================================================
// CoreLangCompiler Class
// ============================================================================
class CoreLangCompiler {
private:
    Lexer lexer;
    Parser parser;
    CodeGenerator codeGen;
    std::vector<std::string> errors;
    
public:
    bool compile(const std::string& source, std::string& outputAsm, std::string& errorMsg) {
        errors.clear();
        outputAsm.clear();
        
        std::vector<Token> tokens = lexer.tokenize(source);
        
        for(const auto& token : tokens) {
            if(token.type == TOK_ERROR) {
                errorMsg = "Lexer error: " + token.value;
                return false;
            }
        }
        
        ASTNode* ast = parser.parse(tokens);
        errors = parser.getErrors();
        
        if(!errors.empty()) {
            errorMsg = "Parser errors:\n";
            for(const auto& err : errors) {
                errorMsg += err + "\n";
            }
            delete ast;
            return false;
        }
        
        outputAsm = codeGen.generate(ast);
        
        delete ast;
        return true;
    }
};

// ============================================================================
// Main Function
// ============================================================================
int main(int argc, char* argv[]) {
    std::cout << "CoreLang to x86 Compiler - With Memory Management, File System, Process Scheduler, PS/2 Mouse, and VESA/VBE Graphics\n";
    std::cout << "EXT2/EXT3/EXT4 File System Support Added!\n";
    std::cout << "PS/2 Mouse Support Added!\n";
    std::cout << "VESA/VBE Graphics Support Added!\n";
    std::cout << "Heap Management: malloc/free/calloc/realloc\n";
    std::cout << "FAT12 File System: file_open/close/read/write/seek/tell/delete/exists\n";
    std::cout << "EXT2/EXT3/EXT4 File System: ext2_mount/open/read/write/close/mkdir/stat/unlink/rename\n";
    std::cout << "Process Scheduler: Round-robin with priority-based preemption\n";
    std::cout << "PS/2 Mouse: mouse_init, mouse_get_x, mouse_get_y, mouse_get_buttons, mouse_get_dx, mouse_get_dy, mouse_set_position, mouse_set_cursor\n";
    std::cout << "VESA/VBE Graphics: vesa_init, vesa_set_mode, vesa_draw_pixel, vesa_draw_rect, vesa_draw_line, vesa_draw_char, vesa_draw_string, vesa_clear, vesa_swap_buffers, vesa_get_width, vesa_get_height, vesa_get_bpp\n";
    std::cout << "===========================================================\n\n";
    
    if(argc < 2) {
        std::cout << "Usage: " << argv[0] << " <input.core> [output.asm]\n";
        std::cout << "\nMemory Management Functions:\n";
        std::cout << "  ptr ptr = malloc(size)      // Allocate memory\n";
        std::cout << "  free(ptr)                   // Free memory\n";
        std::cout << "  ptr ptr = calloc(count, size) // Zero-initialized\n";
        std::cout << "  ptr ptr = realloc(ptr, new_size) // Resize\n";
        std::cout << "  int used = heap_used()      // Get used memory\n";
        std::cout << "  int free = heap_free()      // Get free memory\n";
        std::cout << "  heap_dump()                 // Debug dump\n";
        std::cout << "\nFAT12 File System Functions:\n";
        std::cout << "  int fd = file_open(string name, string mode)  // mode: \"r\", \"w\", \"rw\"\n";
        std::cout << "  file_close(int fd)\n";
        std::cout << "  int bytes = file_read(int fd, ptr buffer, int size)\n";
        std::cout << "  int bytes = file_write(int fd, ptr buffer, int size)\n";
        std::cout << "  int pos = file_seek(int fd, int pos)\n";
        std::cout << "  int pos = file_tell(int fd)\n";
        std::cout << "  file_delete(string name)\n";
        std::cout << "  int exists = file_exists(string name)\n";
        std::cout << "\nEXT2/EXT3/EXT4 File System Functions:\n";
        std::cout << "  int result = ext2_mount(string device, string mount_point)  // Mount partition\n";
        std::cout << "  int fd = ext2_open(string path, int flags)  // flags: O_RDONLY, O_WRONLY, O_RDWR\n";
        std::cout << "  int bytes = ext2_read(int fd, ptr buffer, int size)  // Read from file\n";
        std::cout << "  int bytes = ext2_write(int fd, ptr buffer, int size) // Write to file\n";
        std::cout << "  int result = ext2_close(int fd)  // Close file\n";
        std::cout << "  int result = ext2_mkdir(string path, int mode)  // Create directory\n";
        std::cout << "  int result = ext2_stat(string path, ptr statbuf)  // Get file stats\n";
        std::cout << "  int result = ext2_unlink(string path)  // Delete file\n";
        std::cout << "  int result = ext2_rename(string old_path, string new_path)  // Rename/move\n";
        std::cout << "\nProcess Scheduler Functions:\n";
        std::cout << "  int pid = create_process(func f, string name, int priority)  // Create process\n";
        std::cout << "  int tid = create_thread(func f, int parent_pid, int priority) // Create thread\n";
        std::cout << "  yield()                              // Voluntarily yield CPU\n";
        std::cout << "  sleep(int ms)                        // Sleep for milliseconds\n";
        std::cout << "  int status = wait_pid(int pid)      // Wait for process to exit\n";
        std::cout << "  kill_pid(int pid)                    // Terminate process\n";
        std::cout << "  int pid = getpid()                   // Get current process ID\n";
        std::cout << "  int ppid = getppid()                 // Get parent process ID\n";
        std::cout << "  set_priority(int pid, int priority)  // Change process priority\n";
        std::cout << "\nPS/2 Mouse Functions:\n";
        std::cout << "  mouse_init()                         // Initialize PS/2 mouse\n";
        std::cout << "  int x = mouse_get_x()                // Get current X position (pixels)\n";
        std::cout << "  int y = mouse_get_y()                // Get current Y position (pixels)\n";
        std::cout << "  int buttons = mouse_get_buttons()    // Bit0: left, bit1: right, bit2: middle\n";
        std::cout << "  int dx = mouse_get_dx()              // Get X movement delta\n";
        std::cout << "  int dy = mouse_get_dy()              // Get Y movement delta\n";
        std::cout << "  mouse_set_position(int x, int y)     // Set cursor position\n";
        std::cout << "  mouse_set_cursor(ptr sprite_data, int width, int height) // Custom cursor\n";
        std::cout << "\nVESA/VBE Graphics Functions:\n";
        std::cout << "  int result = vesa_init()             // Initialize VESA, returns 1 on success\n";
        std::cout << "  int result = vesa_set_mode(int width, int height, int bpp) // Set video mode\n";
        std::cout << "  vesa_draw_pixel(int x, int y, int color) // Draw pixel at (x,y)\n";
        std::cout << "  vesa_draw_rect(int x, int y, int w, int h, int color) // Draw filled rectangle\n";
        std::cout << "  vesa_draw_line(int x1, int y1, int x2, int y2, int color) // Draw line (Bresenham)\n";
        std::cout << "  vesa_draw_char(int x, int y, char c, int fg, int bg) // Draw character\n";
        std::cout << "  vesa_draw_string(int x, int y, string s, int fg, int bg) // Draw string\n";
        std::cout << "  vesa_clear(int color)                 // Clear screen with color\n";
        std::cout << "  vesa_swap_buffers()                  // Swap front/back buffers\n";
        std::cout << "  int width = vesa_get_width()         // Get screen width\n";
        std::cout << "  int height = vesa_get_height()       // Get screen height\n";
        std::cout << "  int bpp = vesa_get_bpp()             // Get bits per pixel\n";
        std::cout << "\nExample:\n";
        std::cout << "  func graphics_demo():\n";
        std::cout << "      vesa_init()\n";
        std::cout << "      vesa_clear(0x000000)  // Black\n";
        std::cout << "      vesa_draw_rect(10, 10, 100, 100, 0xFF0000)  // Red square\n";
        std::cout << "      vesa_draw_line(0, 0, 200, 200, 0x00FF00)  // Green diagonal line\n";
        std::cout << "      vesa_draw_string(50, 150, \"Hello VESA!\", 0xFFFFFF, 0x000000)\n";
        std::cout << "      vesa_swap_buffers()\n";
        std::cout << "      return 0\n";
        std::cout << "\nTo build:\n";
        std::cout << "  nasm -f elf32 output.asm -o output.o\n";
        std::cout << "  ld -m elf_i386 -Ttext 0x100000 -o kernel.bin output.o\n";
        std::cout << "  qemu-system-i386 -hda ext2.img -kernel kernel.bin\n";
        return 1;
    }
    
    std::string inputFile = argv[1];
    std::string outputFile = (argc >= 3) ? argv[2] : "output.asm";
    
    std::ifstream inFile(inputFile);
    if(!inFile.is_open()) {
        std::cerr << "Error: Cannot open input file: " << inputFile << std::endl;
        return 1;
    }
    
    std::stringstream buffer;
    buffer << inFile.rdbuf();
    std::string source = buffer.str();
    inFile.close();
    
    std::cout << "Compiling: " << inputFile << std::endl;
    std::cout << "Source size: " << source.length() << " bytes\n\n";
    
    CoreLangCompiler compiler;
    std::string outputAsm;
    std::string errorMsg;
    
    if(compiler.compile(source, outputAsm, errorMsg)) {
        std::ofstream outFile(outputFile);
        if(!outFile.is_open()) {
            std::cerr << "Error: Cannot create output file: " << outputFile << std::endl;
            return 1;
        }
        
        outFile << outputAsm;
        outFile.close();
        
        std::cout << "Compilation successful!\n";
        std::cout << "Assembly output written to: " << outputFile << "\n\n";
        std::cout << "To create an EXT2 filesystem image:\n";
        std::cout << "  dd if=/dev/zero of=ext2.img bs=1024 count=102400\n";
        std::cout << "  mkfs.ext2 ext2.img\n";
        std::cout << "  mkdir /tmp/mnt\n";
        std::cout << "  sudo mount -o loop ext2.img /tmp/mnt\n";
        std::cout << "  echo \"Test data\" > /tmp/mnt/test.txt\n";
        std::cout << "  sudo umount /tmp/mnt\n";
        std::cout << "\nTo build kernel.bin and run:\n";
        std::cout << "  nasm -f elf32 " << outputFile << " -o output.o\n";
        std::cout << "  ld -m elf_i386 -Ttext 0x100000 -o kernel.bin output.o\n";
        std::cout << "  qemu-system-i386 -hda ext2.img -kernel kernel.bin\n";
        
        return 0;
    } else {
        std::cerr << "Compilation failed!\n";
        std::cerr << errorMsg << std::endl;
        return 1;
    }
}