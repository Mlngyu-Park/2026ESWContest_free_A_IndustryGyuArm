#!/usr/bin/env python3
"""Gemma 3n E4B 를 ARM STUDIO 전용으로 깎는다 (LoRA)

LoRA = 모델 전체를 다시 학습하지 않고 '작은 보정층' 만 얹는 방식.
  - 원본 15.7GB 를 건드리지 않는다 (되돌리기 쉽다)
  - 학습이 훨씬 빠르고 메모리도 적게 쓴다
  - 결과물이 수십 MB 라 옮기기 쉽다

토큰 없이 받을 수 있는 unsloth 미러를 쓴다(공식 google/ 은 로그인 필요).
"""
import json, os, sys, torch
from datasets import Dataset
from transformers import AutoTokenizer, AutoModelForCausalLM, TrainingArguments
from peft import LoraConfig, get_peft_model
from trl import SFTTrainer

MODEL = os.environ.get('BASE_MODEL', 'unsloth/gemma-3n-E4B-it')
OUT   = os.environ.get('OUT_DIR', 'out')
DATA  = 'llm_train.jsonl'

def main():
    print(f"[모델] {MODEL}")
    print(f"[GPU ] {torch.cuda.get_device_name(0) if torch.cuda.is_available() else '없음'}")

    rows = [json.loads(l) for l in open(DATA, encoding='utf-8') if l.strip()]
    print(f"[데이터] {len(rows):,} 줄")

    tok = AutoTokenizer.from_pretrained(MODEL)
    if tok.pad_token is None:
        tok.pad_token = tok.eos_token

    def fmt(r):
        return {"text": tok.apply_chat_template(r["messages"], tokenize=False)}
    ds = Dataset.from_list(rows).map(fmt, remove_columns=["messages"])

    model = AutoModelForCausalLM.from_pretrained(
        MODEL, torch_dtype=torch.bfloat16, device_map="auto")
    model.config.use_cache = False

    # LoRA: 주의(attention) 층에만 작은 보정을 붙인다
    model = get_peft_model(model, LoraConfig(
        r=16, lora_alpha=32, lora_dropout=0.05, bias="none",
        task_type="CAUSAL_LM",
        target_modules=["q_proj","k_proj","v_proj","o_proj"]))
    model.print_trainable_parameters()

    trainer = SFTTrainer(
        model=model, train_dataset=ds,
        args=TrainingArguments(
            output_dir=OUT,
            num_train_epochs=3,
            per_device_train_batch_size=2,
            gradient_accumulation_steps=8,
            learning_rate=2e-4,
            bf16=True,
            logging_steps=20,
            save_strategy="epoch",
            save_total_limit=2,
            report_to=[],
            warmup_ratio=0.05,
            lr_scheduler_type="cosine",
        ),
    )
    trainer.train()
    model.save_pretrained(f"{OUT}/lora")
    tok.save_pretrained(f"{OUT}/lora")
    print(f"\n완료 — {OUT}/lora 에 저장했습니다.")
    print("다음: LoRA 를 원본에 합쳐 GGUF 로 바꾸면 지금 앱이 바로 씁니다.")

if __name__ == '__main__':
    main()
