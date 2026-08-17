// --- CONFIGURATION ---
// Automatically targets the host running the page, specifying port 9999 and the /ws endpoint.
// // 🌟 Add the STUN servers to the frontend PeerConnection
const iceConfig = {
    iceServers: [
        { urls: "stun:stun.l.google.com:19302" },
        { urls: "stun:stun1.l.google.com:19302" }
    ]
};

// Update your initialization line:
let pc = new RTCPeerConnection(iceConfig);
const signalingUrl = `ws://${window.location.host}/ws`; 
let ws; // WebSocket

// --- DOM ELEMENTS ---
const callBtn = document.getElementById('playVideoBtn');
const remoteVideo = document.getElementById('remoteVideo');

// 🌟 Add this to track the connection lifecycle
pc.oniceconnectionstatechange = () => {
    console.log("🔮 ICE Connection State changed to:", pc.iceConnectionState);
};

pc.onicegatheringstatechange = () => {
    console.log("📡 ICE Gathering State changed to:", pc.iceGatheringState);
};

pc.onsignalingstatechange = () => {
    console.log("🔑 Signaling State changed to:", pc.signalingState);
};

// --- 1. SIGNALING SETUP (WebSocket) ---
function connectSignaling() {
    ws = new WebSocket(signalingUrl);

    ws.onopen = () => {
        console.log("Connected to FastAPI signaling server.");
        callBtn.disabled = false; // Enable the button only when signaling is ready
    };

    ws.onmessage = async (event) => {
        const message = JSON.parse(event.data);
        console.log("Received message from server:", message.type);

        if (message.sdp) {
            // The Python server has processed our Offer and sent back an Answer
            if (message.type === 'answer') {
                await pc.setRemoteDescription(new RTCSessionDescription(message));
                console.log("Remote description (Answer) set. WebRTC connection establishing...");
            }
        } else if (message.type === 'icecandidate' && message.candidate) {
            // aiortc doesn't always send trickle ICE candidates, but if it does, handle them
            try {
                await pc.addIceCandidate(new RTCIceCandidate(message.candidate));
                console.log("Added remote ICE candidate.");
            } catch (e) {
                console.error("Error adding ICE candidate:", e);
            }
        }
    };

    ws.onclose = () => {
        console.warn("Disconnected from signaling server.");
        callBtn.disabled = true;
    };

    ws.onerror = (err) => {
        console.error("WebSocket error:", err);
    };
}

// --- 2. STARTING THE WEBRTC CALL ---
async function startStream() {
    // Disable button to prevent multiple clicks causing glare
    callBtn.disabled = true; 
    
    // Initialize WebRTC Peer Connection
    pc = new RTCPeerConnection({
        iceServers: [{ urls: "stun:stun.l.google.com:19302" }] // Standard public STUN server
    });

    // CRUCIAL FOR AIORTC: 
    // Since we are not sending camera data from the browser, we must explicitly 
    // tell aiortc that we want to receive video via a transceiver.
    pc.addTransceiver('video', { direction: 'recvonly' });

    // Handle incoming video stream from Python/RealSense
    pc.ontrack = (event) => {
      console.log("Incoming track detected:", event.track.kind);
      if (event.track.kind === 'video') {
          const el = document.getElementById('video');
          el.srcObject = event.streams[0];
          
          // Force playback and handle autoplay restrictions
          el.play().catch(error => {
              console.warn("Autoplay prevented by browser. Click 'Stream' to force play.", error);
          });
      }

    };

    // Gather ICE candidates and send them to Python
    pc.onicecandidate = (event) => {
        if (event.candidate) {
            ws.send(JSON.stringify({
                type: 'icecandidate',
                candidate: event.candidate
            }));
        }
    };

    // Create the Offer
    try {
        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);

        // Send the Offer via WebSocket to FastAPI
        console.log("Sending Offer to Python server...");
        ws.send(JSON.stringify({
            sdp: pc.localDescription.sdp,
            type: pc.localDescription.type
        }));
    } catch (error) {
        console.error("Error creating or sending offer:", error);
    }
}

// --- INITIALIZATION ---
// Bind the button click to the stream logic
callBtn.addEventListener('click', startStream);

// Start the WebSocket connection when the script loads
connectSignaling();

