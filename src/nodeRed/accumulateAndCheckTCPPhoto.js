// Retrieve existing buffer and started flag from flow context
let accumulated = flow.get('accumulatedBuffer') || Buffer.alloc(0);
let started = flow.get('photoTransferStarted') || false;
let lastReceiveTime = flow.get('lastReceiveTime') || 0;

// Constants
const TIMEOUT_MS = 20000; // 20 seconds timeout

// Ensure incoming is a Buffer first
let incoming = msg.payload;
if (!Buffer.isBuffer(incoming)) {
    incoming = Buffer.from(incoming);
}

// Check for timeout if we have started receiving data
const currentTime = Date.now();
if (started && accumulated.length > 0 && (currentTime - lastReceiveTime > TIMEOUT_MS)) {
    node.warn('Timeout: No data received for ' + TIMEOUT_MS/1000 + ' seconds. Resetting buffer.');
    accumulated = Buffer.alloc(0);
    started = false;
    flow.set('accumulatedBuffer', accumulated);
    flow.set('photoTransferStarted', false);
    flow.set('lastReceiveTime', 0);
    
    // Check if current chunk has start marker
    for (let i = 0; i <= incoming.length - 4; i++) {
        if (incoming[i] === 0x00 &&
            incoming[i + 1] === 0x00 && 
            incoming[i + 2] === 0x00 && 
            incoming[i + 3] === 0x00) {
            
            // Found marker in current chunk after timeout
            started = true;
            flow.set('photoTransferStarted', true);
            // Skip the marker (i + 4)
            incoming = incoming.slice(i + 4);
            node.warn('Found start marker in new chunk after timeout');
            break;
        }
    }
    if (!started) {
        return null;
    }
}

// Update last receive time
flow.set('lastReceiveTime', Date.now());

// If we haven't started yet, look for the beginning marker
if (!started) {
    // Check if this chunk contains the beginning marker
    for (let i = 0; i <= incoming.length - 4; i++) {
        if (incoming[i] === 0x00 &&
            incoming[i + 1] === 0x00 && 
            incoming[i + 2] === 0x00 && 
            incoming[i + 3] === 0x00) {
            
            started = true;
            flow.set('photoTransferStarted', true);
            // Skip the marker (i + 4)
            incoming = incoming.slice(i + 4);
            node.warn('Found beginning marker, starting accumulation');
            break;
        }
    }
    
    if (!started) {
        node.warn('Waiting for start marker...');
        return null;
    }
}

// Concatenate
accumulated = Buffer.concat([accumulated, incoming]);
flow.set('accumulatedBuffer', accumulated);

// Debug sizes
node.warn(`Received chunk: ${incoming.length} bytes, total so far: ${accumulated.length}`);

// Check for end marker (0xFF 0xFF 0xFF 0xFF)
if (accumulated.length >= 4) {
    const last4 = accumulated.slice(-4);
    
    if (last4[0] === 0xFF && last4[1] === 0xFF && 
        last4[2] === 0xFF && last4[3] === 0xFF) {
        
        node.warn('Found end marker; emitting photo data');

        // Remove the end marker from the final output
        msg.payload = accumulated.slice(0, -4);

        // Set headers for JPEG
        msg.headers = {
            'Content-Type': 'image/jpeg'
        };

        // Reset all accumulation state
        flow.set('accumulatedBuffer', Buffer.alloc(0));
        flow.set('photoTransferStarted', false);
        flow.set('lastReceiveTime', 0);

        return msg;
    }
}

// Not complete yet, keep accumulating
return null;