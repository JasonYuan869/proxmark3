local MIME_TYPE = "application/vnd.wfa.wsc"

copyright = "Copyright (c) 2026 Jason Yuan"
author = "Jason Yuan"
version = "v1.0.0"
desc = [[
Create a Wi-Fi Simple Configuration NDEF record and write it to an
NDEF-formatted MIFARE Ultralight / NTAG tag.
]]
usage = [[
script run wifi_ndef -s <ssid> [-p <password>] [-a <security>] [-k <tag-pwd>] [-v] [-n]
]]
arguments = [[
    -h, --help            this help
    -s, --ssid            Wi-Fi network name (1-32 UTF-8 bytes)
    -p, --password        Wi-Fi password (omit for an open network)
    -a, --auth            open, wpa, wpa2, or mixed (default: wpa2)
    -k, --tag-password    optional 4-byte NTAG password (8 hex symbols)
    -v, --verify          read and decode the NDEF message after writing
    -n, --dry-run         print the generated command without writing
]]
example = [[
    script run wifi_ndef -s "My WiFi" -p "correct horse battery staple"
    script run wifi_ndef -s Guest -a open -v
    script run wifi_ndef -s Legacy -p secret123 -a mixed
    script run wifi_ndef -s Private -p secret123 -k FFFFFFFF -v
]]

local SECURITY = {
    open  = { auth = 0x0001, encryption = 0x0001 },
    wpa   = { auth = 0x0002, encryption = 0x0004 },
    wpa2  = { auth = 0x0020, encryption = 0x0008 },
    mixed = { auth = 0x0022, encryption = 0x000C },
}

local function help()
    print(copyright)
    print(author)
    print(version)
    print(desc)
    print("Usage")
    print(usage)
    print("Arguments")
    print(arguments)
    print("Examples")
    print(example)
end

-- The stock getopt module splits on whitespace before processing options.
-- This small tokenizer preserves quoted SSIDs and passwords.
local function tokenize(input)
    local tokens = {}
    local chars = {}
    local quote = nil
    local escaped = false
    local started = false

    local function finish_token()
        if started then
            tokens[#tokens + 1] = table.concat(chars)
            chars = {}
            started = false
        end
    end

    for i = 1, #input do
        local ch = input:sub(i, i)
        if escaped then
            chars[#chars + 1] = ch
            escaped = false
            started = true
        elseif ch == "\\" and quote ~= "'" then
            escaped = true
            started = true
        elseif quote then
            if ch == quote then
                quote = nil
            else
                chars[#chars + 1] = ch
            end
            started = true
        elseif ch == "\"" or ch == "'" then
            quote = ch
            started = true
        elseif ch:match("%s") then
            finish_token()
        else
            chars[#chars + 1] = ch
            started = true
        end
    end

    if escaped then
        chars[#chars + 1] = "\\"
    end
    if quote then
        return nil, "unterminated quoted argument"
    end
    finish_token()
    return tokens
end

local function parse_args(input)
    local argv, err = tokenize(input or "")
    if not argv then return nil, err end

    local opts = {
        auth = "wpa2",
        verify = false,
        dry_run = false,
    }

    local value_options = {
        ["-s"] = "ssid", ["--ssid"] = "ssid",
        ["-p"] = "password", ["--password"] = "password",
        ["-a"] = "auth", ["--auth"] = "auth",
        ["-k"] = "tag_password", ["--tag-password"] = "tag_password",
    }

    local i = 1
    while i <= #argv do
        local arg = argv[i]
        local name = value_options[arg]
        if name then
            if i == #argv then
                return nil, arg .. " requires a value"
            end
            opts[name] = argv[i + 1]
            i = i + 2
        elseif arg == "-h" or arg == "--help" then
            opts.help = true
            i = i + 1
        elseif arg == "-v" or arg == "--verify" then
            opts.verify = true
            i = i + 1
        elseif arg == "-n" or arg == "--dry-run" then
            opts.dry_run = true
            i = i + 1
        else
            return nil, "unknown argument: " .. arg
        end
    end
    return opts
end

local function be16(value)
    return string.char(math.floor(value / 256), value % 256)
end

local function attribute(id, value)
    return be16(id) .. be16(#value) .. value
end

local function to_hex(value)
    return (value:gsub(".", function(ch)
        return string.format("%02X", string.byte(ch))
    end))
end

local function valid_network_key(value)
    local length = #value
    if length >= 8 and length <= 63 then return true end
    return length == 64 and value:match("^[0-9A-Fa-f]+$") ~= nil
end

local function validate(opts)
    if not opts.ssid or #opts.ssid == 0 then
        return nil, "SSID is required"
    end
    if #opts.ssid > 32 then
        return nil, "SSID must be at most 32 UTF-8 bytes"
    end

    opts.auth = opts.auth:lower()
    if not SECURITY[opts.auth] then
        return nil, "security must be open, wpa, wpa2, or mixed"
    end

    opts.password = opts.password or ""
    if opts.auth == "open" then
        if #opts.password ~= 0 then
            return nil, "an open network must not include a password"
        end
    elseif not valid_network_key(opts.password) then
        return nil, "WPA/WPA2 passwords must be 8-63 bytes, or 64 hex symbols"
    end

    if opts.tag_password then
        opts.tag_password = opts.tag_password:upper()
        if not opts.tag_password:match("^[0-9A-F]+$") or #opts.tag_password ~= 8 then
            return nil, "tag password must be exactly 8 hex symbols"
        end
    end
    return true
end

local function build_ndef(opts)
    local security = SECURITY[opts.auth]
    local credential = table.concat({
        attribute(0x1026, "\x01"),
        attribute(0x1045, opts.ssid),
        attribute(0x1003, be16(security.auth)),
        attribute(0x100F, be16(security.encryption)),
        attribute(0x1027, opts.password),
        attribute(0x1020, string.rep("\xFF", 6)),
    })
    local payload = attribute(0x104A, "\x10")
        .. attribute(0x100E, credential)

    if #payload > 255 then
        return nil, "Wi-Fi payload is too large for a short NDEF record"
    end

    -- MB=1, ME=1, SR=1, TNF=2 (MIME media record).
    return string.char(0xD2, #MIME_TYPE, #payload)
        .. MIME_TYPE
        .. payload
end

local function main(input)
    local opts, err = parse_args(input)
    if not opts then
        print("ERROR: " .. err)
        print(usage)
        return
    end
    if opts.help then return help() end

    local ok
    ok, err = validate(opts)
    if not ok then
        print("ERROR: " .. err)
        return
    end

    local ndef
    ndef, err = build_ndef(opts)
    if not ndef then
        print("ERROR: " .. err)
        return
    end

    local command = "hf mfu ndefwrite -d " .. to_hex(ndef)
    if opts.tag_password then
        command = command .. " -k " .. opts.tag_password
    end

    if opts.dry_run then
        print(command)
        return
    end

    print(string.format("Writing %s Wi-Fi NDEF record for SSID %q", opts.auth, opts.ssid))
    core.console(command)
    if opts.verify then
        core.console("hf mfu ndefread -v")
    end
end

main(args)
