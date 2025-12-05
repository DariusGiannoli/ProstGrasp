'use client';

import Image from "next/image";
import { useEffect, useState } from "react";

export default function Home() {
  const [benderTranscript, setBenderTranscript] = useState("");
  const [isListening, setIsListening] = useState(false);
  const [isStopping, setIsStopping] = useState(false);
  const [isTalking, setIsTalking] = useState(false);
  const [showMouthOpen, setShowMouthOpen] = useState(false);

  const stopInference = async () => {
    try {
      setIsStopping(true);
      await fetch("http://localhost:5051/shutdown", { method: "POST" });
      console.log("🛑 Shutdown signal sent!");
    } catch (err) {
      console.error("Failed to send shutdown", err);
    } finally {
      setTimeout(() => setIsStopping(false), 1000);
    }
  };

  useEffect(() => {
    const ws = new WebSocket("ws://localhost:5050");

    ws.onopen = () => {
      console.log("✅ WebSocket connected");
    };

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);

      if (data.benderTranscriptReset) {
        setBenderTranscript("");
        setTimeout(() => {
          setIsTalking(true);
        }, 1350); // 🕑 Ajout du délai de 2 secondes ici
      }

      if (data.benderTranscriptAppend) {
        setBenderTranscript((prev) => prev + data.benderTranscriptAppend);
      }

      if (data.doneSpeaking) {
        setIsTalking(false);
      }

      if (data.listening !== undefined) {
        setIsListening(data.listening);
      }
    };

    ws.onclose = () => {
      console.log("❌ WebSocket disconnected");
    };

    return () => ws.close();
  }, []);

  useEffect(() => {
    let interval: NodeJS.Timeout | null = null;

    if (isTalking) {
      interval = setInterval(() => {
        setShowMouthOpen((prev) => !prev);
      }, 300);
    } else {
      setShowMouthOpen(false);
      if (interval) clearInterval(interval);
    }

    return () => {
      if (interval) clearInterval(interval);
    };
  }, [isTalking]);

  return (
    <main className="min-h-screen bg-gradient-to-b from-slate-900 to-slate-800 text-white flex flex-col items-center justify-center relative px-4">
      {/* Logo Phospho */}
      <div className="absolute top-6 left-6">
        <Image src="/phospho-logo.png" alt="Phospho Logo" width={120} height={40} />
      </div>

      {/* Bender */}
      <div className={`mb-6 ${isTalking ? "bender-talking" : ""}`}>
        <Image
          key="bender"
          src={isTalking && showMouthOpen ? "/bender_talking.png" : "/bender_head.png"}
          alt="Bender"
          width={560}
          height={560}
          priority
          className="transition duration-200"
        />
        {/* Preload to avoid flicker on first switch */}
        <Image src="/bender_talking.png" alt="" width={1} height={1} className="hidden" />
      </div>

      {/* Title */}
      <h1 className="text-3xl md:text-4xl font-bold mb-2 text-center">Talk to phosphobot</h1>

      {/* Waveform */}
      <div
        className={`w-full max-w-sm h-16 ${
          isListening ? 'bg-green-500' : 'bg-slate-700'
        } rounded-2xl mb-6 flex items-center justify-center transition-colors`}
      >
        <span className="text-slate-100 italic animate-pulse">
          {isListening ? "🎙️ Listening..." : "🎙️ Waiting..."}
        </span>
      </div>

      {/* Bender Transcript */}
      <div className="text-slate-300 text-lg italic text-center max-w-xl whitespace-pre-wrap px-4 mb-6">
        {benderTranscript}
      </div>

      {/* Stop Inference Button */}
      <button
        onClick={stopInference}
        disabled={isStopping}
        className={`mt-2 px-6 py-3 text-white font-medium rounded-full transition duration-300 ease-in-out shadow-md
          ${
            isStopping
              ? "bg-gray-500 cursor-not-allowed"
              : "bg-gradient-to-r from-red-600 to-red-500 hover:from-red-500 hover:to-red-400 active:scale-95"
          }
        `}
      >
        🛑 {isStopping ? "Stopping..." : "Stop Inference"}
      </button>
    </main>
  );
}
