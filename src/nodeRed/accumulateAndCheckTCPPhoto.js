// Retrieve existing buffer and started flag from flow context
let accumulated = flow.get('accumulatedBuffer') || Buffer.alloc(0);
let started = flow.get('photoTransferStarted') || false;

// Ensure incoming is a Buffer
let incoming = msg.payload;
if (!Buffer.isBuffer(incoming)) {
    incoming = Buffer.from(incoming);
}

// If we haven't started yet, look for the beginning marker
if (!started) {
    // Check if this chunk contains the beginning marker (0x00 0x00 0x00 0x00)
    for (let i = 0; i <= incoming.length - 4; i++) {
        if (incoming[i] === 0x00 && 
            incoming[i + 1] === 0x00 && 
            incoming[i + 2] === 0x00 && 
            incoming[i + 3] === 0x00) {
            
            // Found the marker! Start accumulating from after the marker
            started = true;
            flow.set('photoTransferStarted', true);
            // Only take data after the marker
            incoming = incoming.slice(i + 4);
            node.warn('Found beginning marker, starting accumulation');
            break;
        }
    }
    
    // If we haven't found the start marker, ignore this chunk
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

        // Reset the accumulation state
        flow.set('accumulatedBuffer', Buffer.alloc(0));
        flow.set('photoTransferStarted', false);

        return msg;
    }
}

// Not complete yet, keep accumulating
return null;