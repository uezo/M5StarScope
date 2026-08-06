from datetime import datetime, timedelta
import logging
import os

from dotenv import load_dotenv
load_dotenv()

from aiavatar.sts.stt.openai import OpenAISpeechRecognizer
from aiavatar.sts.llm.openai_responses_websocket import OpenAIResponsesWebSocketService
from aiavatar.sts.tts.openai import OpenAISpeechSynthesizer
from aiavatar.adapter.websocket.server import AIAvatarWebSocketServer

logger = logging.getLogger()
logger.setLevel(logging.INFO)
log_format = logging.Formatter("[%(levelname)s] %(asctime)s : %(message)s")
streamHandler = logging.StreamHandler()
streamHandler.setFormatter(log_format)
logger.addHandler(streamHandler)


OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
if not OPENAI_API_KEY:
    raise RuntimeError(
        "OPENAI_API_KEY is not set. Add it to server/.env or export it "
        "before starting the server."
    )

SYSTEM_PROMPT = """\
You will be asked questions about stars. Answer them based on a location in Tokyo.
Follow the speaking style and output rules below.

## Acknowledgment, opening response, and reasoning
Your output must consist of an acknowledgment or opening response, followed by reasoning, and then the main response.

### Format

<ack>Acknowledgment or opening response</ack>
<think>Reasoning</think>
<answer>Main response</answer>

### Content

- Acknowledgment or opening response: Include a brief affirmative, negative, filler, or similar opening. It must end with punctuation such as a period or exclamation mark.
- Reasoning: State what should be considered and covered in the response. Always reason first, even for a very short response.
- Main response: State what will ultimately be conveyed to the user. Do not repeat the acknowledgment or opening response; continue naturally from it. If the opening response conflicts with the main response, correct course and treat the main response as authoritative.

## Supervisor instructions
Any message beginning with "$" is an instruction from the supervisor program.
Do not respond directly to the supervisor program. Follow its instructions when producing the response for the user.

## Output constraints
- The output will be synthesized as speech, so do not use emoji, symbols, stage directions, URLs, or similar content.
- Do not use Markdown or other formatting syntax.
- Use natural, fluent spoken language.
- Keep the response content to approximately 50 characters or fewer.

## Additional considerations
- The user's input comes from speech recognition and may contain transcription errors. Infer the intended meaning from the context.
"""

# ======================================================
# Pipeline components
# ======================================================

stt = OpenAISpeechRecognizer(
    openai_api_key=OPENAI_API_KEY,
    model="gpt-transcribe",
    debug=True
)

llm = OpenAIResponsesWebSocketService(
    openai_api_key=OPENAI_API_KEY,
    model="gpt-5.6-terra",
    reasoning_effort="none",
    system_prompt=SYSTEM_PROMPT,
    voice_text_tag=["ack", "answer"],
)

tts = OpenAISpeechSynthesizer(
    openai_api_key=OPENAI_API_KEY,
    model="gpt-4o-mini-tts",
    # speaker="alloy",
    instructions="Speak as a guide for users observing the stars. Speak clearly and at a brisk pace.",
    # wav_sample_rate=16000,
    debug=True
)

app_websocket = AIAvatarWebSocketServer(
    stt=stt,
    llm=llm,
    tts=tts,
    response_audio_chunk_size=2048,
    debug=True
)


# ======================================================
# Request hook
# ======================================================

def _format_sky_value(value):
    if value is None:
        return "N/A"
    if isinstance(value, float) and value.is_integer():
        return str(int(value))
    return str(value)


def _format_observed_at(observed_at_utc, utc_offset_minutes):
    if not isinstance(observed_at_utc, str):
        return "Unknown time"

    try:
        observed_at = datetime.fromisoformat(observed_at_utc.replace("Z", "+00:00"))
        observed_at += timedelta(minutes=float(utc_offset_minutes or 0))
        return observed_at.strftime("%Y-%m-%d %H:%M:%S")
    except (TypeError, ValueError):
        return observed_at_utc


def _format_star_list(sky_context):
    fields = sky_context.get("star_fields") or [
        "name",
        "object_type",
        "hip_id",
        "azimuth_deg",
        "altitude_deg",
        "magnitude",
    ]
    field_labels = {
        "object_type": "object type",
        "hip_id": "HIP ID",
        "azimuth_deg": "azimuth (degrees)",
        "altitude_deg": "altitude (degrees)",
    }
    lines = []
    for star in sky_context.get("stars") or []:
        if isinstance(star, dict):
            values = star
        else:
            values = dict(zip(fields, star))

        name = _format_sky_value(values.get("name"))
        details = []
        for field in fields:
            if field == "name":
                continue
            label = field_labels.get(field, field.replace("_", " "))
            details.append(f"{label}: {_format_sky_value(values.get(field))}")
        lines.append(f"- {name} ({', '.join(details)})")

    return "\n".join(lines) or "No observable stars were provided."


def _build_sky_context_text(sky_context, user_text):
    location = sky_context.get("location") or {}
    utc_offset_minutes = location.get("utc_offset_minutes")
    observed_at = _format_observed_at(
        sky_context.get("observed_at_utc"),
        utc_offset_minutes,
    )
    location_name = _format_sky_value(location.get("name"))
    latitude = _format_sky_value(location.get("latitude_deg"))
    longitude = _format_sky_value(location.get("longitude_deg"))
    utc_offset = _format_sky_value(utc_offset_minutes)
    star_list = _format_star_list(sky_context)

    return (
        "$The following is a list of stars observable at the current time and "
        "location. Use it in your response as needed.\n\n"
        f"{observed_at} (UTC) @{location_name} "
        f"(latitude: {latitude} / longitude: {longitude} / "
        f"UTC offset: {utc_offset} minutes), the stars visible in the sky are "
        f"as follows:\n\n{star_list}\n\n"
        f"User input: {user_text or ''}"
    )


@app_websocket.sts.on_before_llm
async def add_sky_context(request):
    metadata = request.metadata or {}
    sky_context = metadata.get("sky_context")
    if isinstance(sky_context, dict):
        request.text = _build_sky_context_text(sky_context, request.text)


# ======================================================
# FastAPI app
# ======================================================

from fastapi import FastAPI
import uvicorn
app = FastAPI()
ws_router = app_websocket.get_websocket_router()
app.include_router(ws_router)
uvicorn.run(app, host="0.0.0.0", port=8000)
