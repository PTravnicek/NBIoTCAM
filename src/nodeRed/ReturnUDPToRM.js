// Parse incoming message
let incoming = msg.payload;

// Convert Buffer to string
let dataStr;
if (Buffer.isBuffer(incoming)) {
    // Convert Buffer directly to string
    dataStr = incoming.toString();
    node.warn("Decoded string: " + dataStr);
} else {
    node.error("Expected Buffer input");
    return null;
}

try {
    // Split measurement and fields
    let parts = dataStr.split(' ');
    if (parts.length !== 2) {
        throw new Error("Invalid message format");
    }

    // Parse fields
    let values = {};
    let fields = parts[1].split(',');
    fields.forEach(field => {
        let [key, value] = field.split('=');
        values[key] = Number(value);
    });

    // Construct final POST body
    let postBody = {
        temperature: 0,
        humidity: 0,
        distance: 0,
        latitude: values.latitude,
        longitude: values.longitude,
        container_id: 5,
        image_status: 1,
        battery_status: values.battery,
        gps_fix_time: 0,
        wake_up_reason: 0
    };

    msg.payload = postBody;
    return msg;

} catch (error) {
    node.error("Failed to parse message: " + error.message);
    node.error("Raw data: " + dataStr);
    return null;
}
