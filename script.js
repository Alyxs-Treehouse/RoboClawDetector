const chatWindow = document.getElementById('chat-window');
const userInput = document.getElementById('user-input');
const sendButton = document.getElementById('send-button');
const voiceInputButton = document.getElementById('voice-input-button');
const statusMessage = document.getElementById('status-message');

// Function to add messages to the chat window
function addMessage(sender, text) {
    const messageDiv = document.createElement('div');
    messageDiv.classList.add('message', sender);
    const paragraph = document.createElement('p');
    paragraph.textContent = text;
    messageDiv.appendChild(paragraph);
    chatWindow.appendChild(messageDiv);
    chatWindow.scrollTop = chatWindow.scrollHeight; // Scroll to bottom
}

// Function to send a text message (user input)
async function sendTextMessage() {
    const text = userInput.value.trim();
    if (text) {
        addMessage('user', text);
        userInput.value = ''; // Clear input

        // Simulate bot response and mechanical arm action
        // In a real scenario, this would involve sending the text to OpenClaw API
        // OpenClaw would then process the question, determine "yes/no",
        // execute arm_actions.py, and send back the text response.

        statusMessage.textContent = '思考中...';

        // Placeholder for OpenClaw API call
        try {
            // This is a placeholder for sending the user's question to OpenClaw
            // and getting back a "yes" or "no" response string.
            // For now, we'll simulate a response.
            const simulatedResponse = await simulateOpenClawResponse(text);
            
            addMessage('bot', simulatedResponse.text);
            speakResponse(simulatedResponse.text); // Speak the bot's response
            statusMessage.textContent = '';
            
            // The mechanical arm action would actually be triggered by OpenClaw
            // but for frontend only demo, we can imagine it happened.
            console.log(`Mechanical arm action for: ${simulatedResponse.text}`);

        } catch (error) {
            console.error('Error communicating with OpenClaw (simulated):', error);
            addMessage('bot', '抱歉，我暂时无法回答您的问题。');
            speakResponse('抱歉，我暂时无法回答您的问题。');
            statusMessage.textContent = '发生错误。';
        }
    }
}

// Simulate OpenClaw response (replace with actual API call)
async function simulateOpenClawResponse(question) {
    return new Promise(resolve => {
        setTimeout(() => {
            const lowerCaseQuestion = question.toLowerCase();
            if (lowerCaseQuestion.includes('男人') && lowerCaseQuestion.includes('死了')) {
                resolve({ text: '是' });
            } else if (lowerCaseQuestion.includes('汤') && lowerCaseQuestion.includes('难喝')) {
                resolve({ text: '否' });
            } else if (lowerCaseQuestion.includes('妻子') || lowerCaseQuestion.includes('老婆')) {
                resolve({ text: '是' }); 
            }
            else {
                const randomResponse = Math.random() < 0.5 ? '是' : '否';
                resolve({ text: randomResponse });
            }
        }, 1500); // Simulate network delay
    });
}

// --- Web Speech API for TTS (Text-to-Speech) ---
let synth = window.speechSynthesis;
let currentUtterance = null;

function speakResponse(text) {
    if (synth.speaking) {
        synth.cancel(); // Stop any current speech
    }
    currentUtterance = new SpeechSynthesisUtterance(text);
    currentUtterance.lang = 'zh-CN'; // Set language to Chinese
    synth.speak(currentUtterance);
}

// --- Event Listeners ---
sendButton.addEventListener('click', sendTextMessage);

userInput.addEventListener('keydown', (event) => {
    if (event.key === 'Enter') {
        sendTextMessage();
    }
});

// Placeholder for Voice Input (will be implemented in next step)
voiceInputButton.addEventListener('click', () => {
    statusMessage.textContent = '语音输入功能待实现...';
});
