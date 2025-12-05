"use client"

import { useState } from "react"
import { Button } from "@/components/ui/button"
import { ThumbsUp, ThumbsDown } from "lucide-react"
import { cn } from "@/lib/utils"
import { fetchWithBaseUrl } from "@/lib/utils"

interface FeedbackButtonsProps {
    aiControlID: string | undefined
    onFeedbackSubmitted?: (isPositive: boolean) => void
    className?: string
}

export function FeedbackButtons({ aiControlID, onFeedbackSubmitted, className }: FeedbackButtonsProps) {
    const [selectedFeedback, setSelectedFeedback] = useState<"positive" | "negative" | null>(null)
    const [isSubmitting, setIsSubmitting] = useState(false)

    const submitFeedback = async (isPositive: boolean) => {
        if (isSubmitting || !aiControlID) return

        try {
            setIsSubmitting(true)

            // Set the feedback state immediately for better UX
            setSelectedFeedback(isPositive ? "positive" : "negative")

            const response = await fetchWithBaseUrl(`/ai-control/feedback`, "POST", {
                feedback: isPositive ? "positive" : "negative",
                ai_control_id: aiControlID,
            })

            // Check if the response is valid
            if (!response) {
                throw new Error("No response from server")
            }

            // Call the callback if provided
            onFeedbackSubmitted?.(isPositive)
        } catch (error) {
            console.error("Error submitting feedback:", error)

            // Reset the feedback state if there was an error
            setSelectedFeedback(null)
        } finally {
            setIsSubmitting(false)
        }
    }

    return (
        <div className={cn("flex items-center gap-2", className)}>
            <Button
                variant="outline"
                size="sm"
                className={cn(
                    "border-gray-200 hover:bg-gray-100 hover:text-gray-900",
                    selectedFeedback === "positive" && "bg-green-50 text-green-600 border-green-200 hover:bg-green-100",
                )}
                disabled={isSubmitting}
                onClick={() => submitFeedback(true)}
                aria-label="Thumbs up"
            >
                <ThumbsUp className="h-4 w-4 mr-1" />
                <span>Good</span>
            </Button>

            <Button
                variant="outline"
                size="sm"
                className={cn(
                    "border-gray-200 hover:bg-gray-100 hover:text-gray-900",
                    selectedFeedback === "negative" && "bg-red-50 text-red-600 border-red-200 hover:bg-red-100",
                )}
                disabled={isSubmitting}
                onClick={() => submitFeedback(false)}
                aria-label="Thumbs down"
            >
                <ThumbsDown className="h-4 w-4 mr-1" />
                <span>Bad</span>
            </Button>
        </div>
    )
}

export default FeedbackButtons
