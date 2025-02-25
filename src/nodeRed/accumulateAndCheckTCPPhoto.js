// Retrieve existing buffer, started flag, and last activity timestamp from flow context
let accumulated = flow.get('accumulatedBuffer') || Buffer.alloc(0);
let started = flow.get('photoTransferStarted') || false;
let lastActivity = flow.get('lastPhotoActivity') || Date.now();

// Update last activity timestamp
flow.set('lastPhotoActivity', Date.now());

// Check for timeout if we've started receiving data
if (started && accumulated.length > 0) {
    const TIMEOUT_MS = 20000; // 20 seconds
    if (Date.now() - lastActivity > TIMEOUT_MS) {
        node.warn('Timeout detected - no data received for 20 seconds. Forcing completion.');
        
        // Remove any partial end marker that might be present
        while (accumulated.length >= 4 && 
               accumulated[accumulated.length - 1] === 0xFF &&
               accumulated[accumulated.length - 2] === 0xFF &&
               accumulated[accumulated.length - 3] === 0xFF &&
               accumulated[accumulated.length - 4] === 0xFF) {
            accumulated = accumulated.slice(0, -4);
        }
        
        // Create message with accumulated data
        msg.payload = accumulated;
        msg.headers = {
            'Content-Type': 'image/jpeg'
        };
        msg.timeout = true; // Flag to indicate timeout completion

        // Reset all state
        flow.set('accumulatedBuffer', Buffer.alloc(0));
        flow.set('photoTransferStarted', false);
        flow.set('lastPhotoActivity', null);

        return msg;
    }
}

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
        flow.set('lastPhotoActivity', null);

        return msg;
    }
}

// Not complete yet, keep accumulating
return null;